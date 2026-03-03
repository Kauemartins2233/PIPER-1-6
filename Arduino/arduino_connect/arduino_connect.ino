#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define DEBUG_DATA 1
#define DEBUG_HZ 0
#define DEG2RAD 0.017453293f
#define LONG_LATERO
//#define LONG_ROLL_CONTROL
/* ---------------------------------------------------------------------------
 *  ESTRUTURA DO CONTROLADOR PI DISCRETO + TERMO DE AMORTECIMENTO (kv * vel)
 *
 *  Implementa:
 *      u[k] = u[k-1] + b0 * e[k] + b1 * e[k-1] - kv * vel[k]
 *
 *  onde:
 *      e[k]   = setpoint - measurement
 *      vel[k] = taxa medida (p, q, r), usada como “derivativo físico”
 * -------------------------------------------------------------------------*/
struct PI_Digital {
  float b0;      // coeficiente b0 (obtido do c2d)
  float b1;      // coeficiente b1 (obtido do c2d)
  float kv;      // ganho do termo de taxa (ex.: Kq * q, Kp * p ou Kr * r)
  float e_prev;  // erro anterior e[k-1]
  float u_prev;  // saída anterior u[k-1]
  float umin;    // saturação mínima
  float umax;    // saturação máxima
};

struct WashoutFilter {
  float b0;
  float b1;
  float a1;
  float x_prev;  // x[k-1]
  float y_prev;  // y[k-1]
};

/* ---------------------------------------------------------------------------
 *  PARÂMETROS DOS CONTROLADORES
 *  (coeficientes discretizados via MATLAB: c2d(PI, Ts=0.01, 'tustin'))
 * -------------------------------------------------------------------------*/
// Velocidade: throttle [0..1]
PI_Digital pi_vel = {
  0.064427f,   // b0
  -0.064373f,  // b1
  0.0f,        // kv
  0.0f, 0.0f,
  0.0f,  // umin
  1.0f   // umax
};



// Altitude: referência de pitch (theta_ref em rad)
PI_Digital pi_alt = {
  0.02224702f,
  -0.02224556,
  0.0f,
  0.0f, 0.0f,
  -20.0f * DEG2RAD,  // umin: -20°
  20.0f * DEG2RAD    // umax:  20°
};

// Pitch (theta) → comando de elevador normalizado [-1..1]
PI_Digital pi_theta = {
  0.6367f,
  -0.6270f,
  0.01f,  // kv: ganho na taxa de pitch (q)
  0.0f, 0.0f,
  -1,
  1
};

#ifdef LONG_ROLL_CONTROL
// Roll (phi): aileron normalizado [-1..1]
PI_Digital pi_phi = {
  0.6367f,
  -0.6270f,
  0.1f,  // kv: ganho na taxa de roll (p)
  0.0f, 0.0f,
  -1,
  1
};

// Yaw (psi):rudder normalizado [-1..1]
PI_Digital pi_psi = {
  0.6367f,
  -0.6270f,
  0.1f,  // kv: ganho na taxa de yaw (r)
  0.0f, 0.0f,
  -1,
  1
};
#endif

#ifdef LONG_LATERO
   
// Roll (phi): aileron normalizado [-1..1]
PI_Digital pi_phi = {
  0.102f,
  -0.1000f,
  0.1f,  // kv: ganho na taxa de roll (p)
  0.0f, 0.0f,
-1,
1
};

// Yaw (psi):rudder normalizado [-1..1]
PI_Digital pi_psi = {
  0.0f,
  0.0f,
  0.1f,  // kv: ganho na taxa de yaw (r)
  0.0f, 0.0f,
-1,
1
};
#endif

/* ---------------------------------------------------------------------------
 *  REFERÊNCIAS
 * -------------------------------------------------------------------------*/
float REF_ALT = 1000.0f;     // [m]
float REF_THETA = -0.1217f;  // [rad]
float REF_VEL = 25.0f;       // [m/s]
float REF_ROLL = 0.0f;       // [rad]
float REF_YAW = 90.0f;       // [deg]
float theta_ref_from_alt = 0.0f;
/* ---------------------------------------------------------------------------
 *  CONFIGURAÇÃO ETHERNET / X-PLANE
 * -------------------------------------------------------------------------*/
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress localIP(192, 168, 2, 2);    // IP fixo do Arduino
IPAddress XPLANE_IP(192, 168, 2, 1);  // IP do PC com X-Plane

const uint16_t LISTEN_PORT = 49010;  // Porta que recebe DATA do X-Plane
const uint16_t XPLANE_PORT = 49000;  // Porta para enviar DREF

EthernetUDP udp;

/* ---------------------------------------------------------------------------
 *  VARIÁVEIS DE ESTADO DOS DADOS RECEBIDOS DO X-PLANE
 * -------------------------------------------------------------------------*/
// (ângulos em graus recebidos)
float pitch_deg = 0.0f;
float roll_deg = 0.0f;
float hdg_mag = 0.0f;
uint16_t rpy_hz = 0;

// taxas angulares em deg/s
float p_deg = 0.0f;
float q_deg = 0.0f;
float r_deg = 0.0f;
uint16_t pqr_hz = 0;

// carregamento g
float n_g = 0.0f;
float a_g = 0.0f;
float s_g = 0.0f;
uint16_t gload_hz = 0;

// posição GPS
float lat = 0.0f;
float lon = 0.0f;
float alt = 0.0f;
uint16_t gps_hz = 0;

// comandos atuais enviados pelo X-Plane
float ail = 0.0f;
float elv = 0.0f;
float rud = 0.0f;
uint16_t cmd_hz = 0;

// throttle atual
float thro = 0.0f;
uint16_t thr_hz = 0;

// velocidade
float vel = 0.0f;
uint16_t vel_hz = 0;

// comandos calculados pelo controlador (normalizados)
float elv_cmd = 0.0f;
float ail_cmd = 0.0f;
float rud_cmd = 0.0f;
float thr_cmd = 0.15f;

// buffers de recepção/envio
static uint8_t buf[300];
static uint8_t bufDREF[509];


WashoutFilter washout_yaw;
/* Índices dos pacotes DATA do X-Plane */
enum Idx {
  SPEED = 3,
  G_LOAD = 4,
  CMD = 8,
  CMD_TRIM = 8,
  PQR = 17,
  RPY = 18,
  GPS = 20,
  THROTTLE = 26,
};

/* ---------------------------------------------------------------------------
 *  FUNÇÕES AUXILIARES PARA LITTLE-ENDIAN
 * -------------------------------------------------------------------------*/
static inline int32_t get_le32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static inline float get_le_f32(const uint8_t* p) {
  uint32_t u = (uint32_t)p[0]
               | ((uint32_t)p[1] << 8)
               | ((uint32_t)p[2] << 16)
               | ((uint32_t)p[3] << 24);
  float f;
  memcpy(&f, &u, sizeof(f));
  return f;
}

/* Nome de cada índice Tabela X-Plane*/
const char* labelForIndex(int idx) {
  switch (idx) {
    case SPEED: return "Speeds";
    case G_LOAD: return "G-load";
    case CMD: return "ail/elv/rud";
    case 13: return "trim";
    case PQR: return "pqr";
    case RPY: return "pitch, roll, headings";
    case GPS: return "lat/lon/alt";
    case THROTTLE: return "Throttle actual";
    default: return "";
  }
}

/* ---------------------------------------------------------------------------
 *  CLAMP
 * -------------------------------------------------------------------------*/
float clamp(float x, float xmin, float xmax) {
  if (x < xmin) return xmin;
  if (x > xmax) return xmax;
  return x;
}

/* ---------------------------------------------------------------------------
 *  ATUALIZAÇÃO DO CONTROLADOR PI DISCRETO + TERMO kv * vel
 * -------------------------------------------------------------------------*/
float pi_update(PI_Digital& pi, float setpoint, float measurement, float vel) {
  float e = setpoint - measurement;

  // u[k] = u[k-1] + b0 * e[k] + b1 * e[k-1] - kv * vel[k]
  float u = pi.u_prev
            + pi.b0 * e
            + pi.b1 * pi.e_prev
            - pi.kv * vel;

  // saturação
  u = clamp(u, pi.umin, pi.umax);

  // atualiza estados
  pi.e_prev = e;
  pi.u_prev = u;

  return u;
}

/* ---------------------------------------------------------------------------
 *  ENVIO DE DREF PARA O X-PLANE
 * -------------------------------------------------------------------------*/
bool sendDREF(const char* dref, float value) {
  // Cabeçalho "DREF\0"
  bufDREF[0] = 'D';
  bufDREF[1] = 'R';
  bufDREF[2] = 'E';
  bufDREF[3] = 'F';
  bufDREF[4] = 0;

  // valor float em little-endian
  uint32_t u;
  memcpy(&u, &value, sizeof(value));
  bufDREF[5] = (uint8_t)(u & 0xFF);
  bufDREF[6] = (uint8_t)((u >> 8) & 0xFF);
  bufDREF[7] = (uint8_t)((u >> 16) & 0xFF);
  bufDREF[8] = (uint8_t)((u >> 24) & 0xFF);

  // string do DREF
  size_t dlen = strnlen(dref, 499);
  memcpy(&bufDREF[9], dref, dlen);
  bufDREF[9 + dlen] = 0;

  udp.beginPacket(XPLANE_IP, XPLANE_PORT);
  size_t written = udp.write(bufDREF, sizeof(bufDREF));
  bool ok = (udp.endPacket() == 1);
  return ok && (written == sizeof(bufDREF));
}

/* ---------------------------------------------------------------------------
 *  ENVIO DOS COMANDOS NORMALIZADOS PARA O X-PLANE
 * -------------------------------------------------------------------------*/
void setControls(float elv_cmd_, float ail_cmd_, float rud_cmd_, float thr_cmd_) {
  // saturação no formato esperado pelo X-Plane
  // elv_cmd_ = constrain(elv_cmd_, -15.0 * DEG2RAD, 15.0 * DEG2RAD) / (15.0 * DEG2RAD);
  // ail_cmd_ = constrain(ail_cmd_, -15.0 * DEG2RAD, 15.0 * DEG2RAD) / 15.0 * DEG2RAD;
  // rud_cmd_ = constrain(rud_cmd_, -15.0 * DEG2RAD, 15.0 * DEG2RAD) / 15.0 * DEG2RAD;
  // thr_cmd_ = constrain(thr_cmd_, 0.0f, 1.0f);
  elv_cmd_ = constrain(elv_cmd_, -1, 1);
  ail_cmd_ = constrain(ail_cmd_, -1, 1);
  rud_cmd_ = constrain(rud_cmd_, -1, 1);
  thr_cmd_ = constrain(thr_cmd_, 0.0f, 1.0f);
  sendDREF("sim/cockpit2/controls/yoke_pitch_ratio", elv_cmd);
  sendDREF("sim/cockpit2/controls/yoke_roll_ratio", ail_cmd);
  sendDREF("sim/cockpit2/controls/yoke_heading_ratio", rud_cmd);
  sendDREF("sim/cockpit2/engine/actuators/throttle_ratio_all", thr_cmd);
}

/* ---------------------------------------------------------------------------
 *  PRINT DOS DADOS DE TELEMETRIA (DEBUG)
 * -------------------------------------------------------------------------*/
void printData() {
  Serial.print(F(" roll: "));
  Serial.print(roll_deg);
  Serial.print(F(" pitch: "));
  Serial.print(pitch_deg);
  Serial.print(F(" yaw: "));
  Serial.print(hdg_mag);

  Serial.print(F(" | P: "));
  Serial.print(p_deg);
  Serial.print(F(" Q: "));
  Serial.print(q_deg);
  Serial.print(F(" R: "));
  Serial.print(r_deg);

  Serial.print(F(" | g_n: "));
  Serial.print(n_g);
  Serial.print(F(" g_a: "));
  Serial.print(a_g);
  Serial.print(F(" g_s: "));
  Serial.print(s_g);

  Serial.print(F(" | lat: "));
  Serial.print(lat);
  Serial.print(F(" lon: "));
  Serial.print(lon);
  Serial.print(F(" alt: "));
  Serial.print(alt);
  Serial.print(F(" vel: "));
  Serial.print(vel);

  Serial.print(F(" | ele_cmd: "));
  Serial.print(elv_cmd);
  Serial.print(F(" ail_cmd: "));
  Serial.print(ail_cmd);
  Serial.print(F(" rud_cmd: "));
  Serial.print(rud_cmd);
  Serial.print(F(" thr_cmd: "));
  Serial.print(thr_cmd);

  // imprimir as referências
  Serial.print(F(" | REF_ALT: "));
  Serial.print(REF_ALT);

  Serial.print(F(" REF_THETA[deg]: "));
  Serial.print(REF_THETA / DEG2RAD);

  Serial.print(F(" REF_VEL: "));
  Serial.print(REF_VEL);

  Serial.print(F(" REF_ROLL[deg]: "));
  Serial.print(REF_ROLL / DEG2RAD);

  Serial.print(F(" REF_YAW[deg]: "));
  Serial.println(REF_YAW);
}

// inicializa coeficientes para dado Tw e Ts
void washout_init(WashoutFilter& f, float Tw, float Ts) {
  float k = 2.0f * Tw / Ts;

  f.b0 = k / (k + 1.0f);
  f.b1 = -k / (k + 1.0f);
  f.a1 = (1.0f - k) / (k + 1.0f);

  f.x_prev = 0.0f;
  f.y_prev = 0.0f;
}

// atualiza o filtro a cada amostra
float washout_update(WashoutFilter& f, float x) {
  // y[k] = -a1*y[k-1] + b0*x[k] + b1*x[k-1]
  float y = -f.a1 * f.y_prev
            + f.b0 * x
            + f.b1 * f.x_prev;

  f.x_prev = x;
  f.y_prev = y;

  return y;
}

/* ---------------------------------------------------------------------------
 *  SETUP
 * -------------------------------------------------------------------------*/
unsigned long last_monitor_time = 0;
unsigned long last_control_time = 0;

void setup() {
  Serial.begin(115200);

  Ethernet.begin(mac, localIP);
  delay(1000);

  Serial.print("Ethernet inicializada. IP: ");
  Serial.println(Ethernet.localIP());

  if (udp.begin(LISTEN_PORT)) {
    Serial.print("UDP ouvindo na porta ");
    Serial.println(LISTEN_PORT);
  } else {
    Serial.println("Falha ao iniciar UDP");
  }

  last_control_time = millis();

  float Ts = 0.01f;  // 100 Hz
  float Tw = 5.0f;   // washout de 5s (ajustável)
  washout_init(washout_yaw, Tw, Ts);
}

/* ---------------------------------------------------------------------------
 *  LOOP PRINCIPAL
 * -------------------------------------------------------------------------*/
void loop() {
  // ------------------------------------------------------------------------
  // 1) Recebe pacote DATA do X-Plane
  // ------------------------------------------------------------------------
  processIncoming();
  // ------------------------------------------------------------------------
  // 2) Loop de controle ~100 Hz (a cada 10 ms)
  // ------------------------------------------------------------------------
  unsigned long now = millis();
  if (now - last_control_time >= 10) {


    //Controle logitudinal + latero direcional
#ifdef LONG_LATERO
    longControl();
    lateroControl();
#endif

//Controle logitudinal+roll
#ifdef LONG_ROLL_CONTROL
    longRollControl();
#endif
    // Envia comandos ao X-Plane
    setControls(elv_cmd, ail_cmd, rud_cmd, thr_cmd);

    last_control_time = now;
  }

  debug(now);

  delay(1);  // pequeno yield para não travar o loop
}

void longControl() {
  // converte para radianos
  float pitch = pitch_deg * DEG2RAD;
  float q = q_deg * DEG2RAD;

  // Controle de velocidade: throttle
  thr_cmd = pi_update(pi_vel, REF_VEL, vel, 0.0f);

  // Controle de altitude: gera referência de pitch (em rad)
  theta_ref_from_alt = pi_update(pi_alt, REF_ALT, alt, 0.0f) + REF_THETA;

  // Controle de pitch: elevator
  elv_cmd = pi_update(pi_theta, theta_ref_from_alt, pitch, q);
}
void lateroControl() {
  // converte para radianos
  float roll = roll_deg * DEG2RAD;
  float yaw = hdg_mag * DEG2RAD;

  float p = p_deg * DEG2RAD;
  float r = r_deg * DEG2RAD;

  // passa r no washout
  float r_wash = washout_update(washout_yaw, r_wash);

  // Controle de yaw: rudder (referência de yaw em rad)
  float ref_yaw_rad = REF_YAW * DEG2RAD;
  rud_cmd = pi_update(pi_psi, 0.0, 0.0, r_wash);

  float ref_roll = 0.015 * (ref_yaw_rad - yaw) - roll;
  // Controle de roll:aileron
  ail_cmd = pi_update(pi_phi, ref_roll, roll, p);
}

void longRollControl() {
  // converte para radianos
  float roll = roll_deg * DEG2RAD;
  float pitch = pitch_deg * DEG2RAD;
  float yaw = hdg_mag * DEG2RAD;

  float p = p_deg * DEG2RAD;
  float q = q_deg * DEG2RAD;
  float r = r_deg * DEG2RAD;

  // Controle de velocidade: throttle
  thr_cmd = pi_update(pi_vel, REF_VEL, vel, 0.0f);

  // Controle de altitude: gera referência de pitch (em rad)
  float theta_ref_from_alt = pi_update(pi_alt, REF_ALT, alt, 0.0f) + REF_THETA;

  // Controle de pitch: elevator
  elv_cmd = pi_update(pi_theta, theta_ref_from_alt, pitch, q);

  // Controle de roll:aileron
  ail_cmd = pi_update(pi_phi, REF_ROLL, roll, p);

  // Controle de yaw: rudder (referência de yaw em rad)
  float ref_yaw_rad = REF_YAW * DEG2RAD;
  rud_cmd = pi_update(pi_psi, ref_yaw_rad, yaw, r);
}

void processIncoming() {
  int pkt = udp.parsePacket();
  if (pkt > 0) {

    int n = udp.read(buf, sizeof(buf));
    if (n >= 5) {

      // verifica cabeçalho "DATA\0"
      if (buf[0] == 'D' && buf[1] == 'A' && buf[2] == 'T' && buf[3] == 'A') {

        // cada bloco: 36 bytes = 4(idx) + 8*4(floats)
        for (int i = 5; i + 36 <= n; i += 36) {
          int idx = get_le32(&buf[i]);
          float v[8];
          for (int k = 0; k < 8; ++k) {
            v[k] = get_le_f32(&buf[i + 4 + 4 * k]);
          }

          switch (idx) {
            case RPY:
              pitch_deg = v[0];
              roll_deg = v[1];
              hdg_mag = v[2];
              rpy_hz++;
              break;

            case PQR:
              p_deg = v[1];
              q_deg = v[0];
              r_deg = v[2];
              pqr_hz++;
              break;

            case G_LOAD:
              n_g = v[4];
              a_g = v[5];
              s_g = v[6];
              gload_hz++;
              break;

            case GPS:
              lat = v[0];
              lon = v[1];
              alt = v[3];
              gps_hz++;
              break;

            case CMD:
              elv = v[0];
              ail = v[1];
              rud = v[2];
              cmd_hz++;
              break;

            case THROTTLE:
              thro = v[0];
              thr_hz++;
              break;

            case SPEED:
              vel = v[3] * 0.5f;
              vel_hz++;
              break;

            default:
              break;
          }
        }

      } else {
        Serial.println(F("Pacote sem cabecalho DATA\\0"));
      }
    }
  }
}

void debug(unsigned long now) {
  // ------------------------------------------------------------------------
  // Monitor serial a cada 1 s
  // ------------------------------------------------------------------------
  if (now - last_monitor_time >= 1000) {
    //Serial.println(F("-----------------------------"));

#if DEBUG_DATA
    printData();
#endif
    // zera contadores de Hz
    rpy_hz = 0;
    pqr_hz = 0;
    gload_hz = 0;
    gps_hz = 0;
    cmd_hz = 0;
    thr_hz = 0;
    vel_hz = 0;

    last_monitor_time = now;
  }
}
