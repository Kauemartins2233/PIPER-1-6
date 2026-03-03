#include <WiFi.h>
#include <WiFiUdp.h>

#define DEBUG_DATA 1
#define DEBUG_HZ 0
#define DEG2RAD 0.017453293f
#define LONG_LATERO
//#define LONG_ROLL_CONTROL

/* ---------------------------------------------------------------------------
 *  CONTROLADOR PI DISCRETO + TERMO kv * vel
 * -------------------------------------------------------------------------*/
struct PI_Digital {
  float b0;
  float b1;
  float kv;
  float e_prev;
  float u_prev;
  float umin;
  float umax;
};

struct WashoutFilter {
  float b0;
  float b1;
  float a1;
  float x_prev;
  float y_prev;
};

/* ---------------------------------------------------------------------------
 *  PARÂMETROS DOS CONTROLADORES
 * -------------------------------------------------------------------------*/
// Velocidade: throttle [0..1]
PI_Digital pi_vel = {
  0.064427f,
  -0.064373f,
  0.0f,
  0.0f, 0.0f,
  0.0f,
  1.0f
};

// Altitude: referência de pitch (theta_ref em rad)
PI_Digital pi_alt = {
  0.02224702f,
  -0.02224556f, 
  0.0f,
  0.0f, 0.0f,
  -20.0f * DEG2RAD,
   20.0f * DEG2RAD
};

// Pitch (theta): comando de elevador
PI_Digital pi_theta = {
  0.6367f,
  -0.6270f,
  0.01f,  // kv (q)
  0.0f, 0.0f,
  -1.0f,
   1.0f
};

#ifdef LONG_ROLL_CONTROL
// Roll (phi): aileron
PI_Digital pi_phi = {
  0.6367f,
  -0.6270f,
  0.1f,  // kv (p)
  0.0f, 0.0f,
  -1.0f,
   1.0f
};

// Yaw (psi): rudder
PI_Digital pi_psi = {
  0.6367f,
  -0.6270f,
  0.1f,  // kv (r)
  0.0f, 0.0f,
  -1.0f,
   1.0f
};
#endif

#ifdef LONG_LATERO
// Roll (phi): aileron
PI_Digital pi_phi = {
  0.102f,
  -0.1000f,
  0.1f,  // kv (p)
  0.0f, 0.0f,
  -1.0f,
   1.0f
};

// Yaw (psi): rudder
PI_Digital pi_psi = {
  0.0f,
  0.0f,
  0.1f,  // kv (r)
  0.0f, 0.0f,
  -1.0f,
   1.0f
};
#endif

/* ---------------------------------------------------------------------------
 *  REFERÊNCIAS
 * -------------------------------------------------------------------------*/
float REF_ALT   = 1000.0f;     // [m]
float REF_THETA = -0.1217f;    // [rad]
float REF_VEL   = 25.0f;       // [m/s]
float REF_ROLL  = 0.0f;        // [rad]
float REF_YAW   = 90.0f;       // [deg]
float theta_ref_from_alt = 0.0f;

/* ---------------------------------------------------------------------------
 *  CONFIG WiFi / X-PLANE (ESP32)
 * -------------------------------------------------------------------------*/
const char* ssid     = "SEU_SSID";      // <<< TROCAR
const char* password = "SUA_SENHA";     // <<< TROCAR
IPAddress XPLANE_IP(192, 168, 143, 65);    // IP do PC com X-Plane

const uint16_t LISTEN_PORT = 49010;
const uint16_t XPLANE_PORT = 49000;

WiFiUDP udp;

/* ---------------------------------------------------------------------------
 *  VARIÁVEIS DE ESTADO
 * -------------------------------------------------------------------------*/
float pitch_deg = 0.0f;
float roll_deg  = 0.0f;
float hdg_mag   = 0.0f;
uint16_t rpy_hz = 0;

float p_deg = 0.0f;
float q_deg = 0.0f;
float r_deg = 0.0f;
uint16_t pqr_hz = 0;

float n_g = 0.0f;
float a_g = 0.0f;
float s_g = 0.0f;
uint16_t gload_hz = 0;

float lat = 0.0f;
float lon = 0.0f;
float alt = 0.0f;
uint16_t gps_hz = 0;

float ail = 0.0f;
float elv = 0.0f;
float rud = 0.0f;
uint16_t cmd_hz = 0;

float thro = 0.0f;
uint16_t thr_hz = 0;

float vel = 0.0f;
uint16_t vel_hz = 0;

float elv_cmd = 0.0f;
float ail_cmd = 0.0f;
float rud_cmd = 0.0f;
float thr_cmd = 0.15f;

static uint8_t buf[300];
static uint8_t bufDREF[509];

WashoutFilter washout_yaw;

enum Idx {
  SPEED    = 3,
  G_LOAD   = 4,
  CMD      = 8,
  CMD_TRIM = 8,
  PQR      = 17,
  RPY      = 18,
  GPS      = 20,
  THROTTLE = 26,
};

/* ---------------------------------------------------------------------------
 *  LITTLE-ENDIAN HELPERS
 * -------------------------------------------------------------------------*/
static inline int32_t get_le32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] |
                  ((uint32_t)p[1] << 8) |
                  ((uint32_t)p[2] << 16) |
                  ((uint32_t)p[3] << 24));
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

/* ---------------------------------------------------------------------------
 *  CLAMP
 * -------------------------------------------------------------------------*/
float clamp(float x, float xmin, float xmax) {
  if (x < xmin) return xmin;
  if (x > xmax) return xmax;
  return x;
}

/* ---------------------------------------------------------------------------
 *  PI DIGITAL
 * -------------------------------------------------------------------------*/
float pi_update(PI_Digital& pi, float setpoint, float measurement, float vel) {
  float e = setpoint - measurement;

  float u = pi.u_prev
            + pi.b0 * e
            + pi.b1 * pi.e_prev
            - pi.kv * vel;

  u = clamp(u, pi.umin, pi.umax);

  pi.e_prev = e;
  pi.u_prev = u;

  return u;
}

/* ---------------------------------------------------------------------------
 *  WASHOUT
 * -------------------------------------------------------------------------*/
void washout_init(WashoutFilter& f, float Tw, float Ts) {
  float k = 2.0f * Tw / Ts;

  f.b0 =  k / (k + 1.0f);
  f.b1 = -k / (k + 1.0f);
  f.a1 = (1.0f - k) / (k + 1.0f);

  f.x_prev = 0.0f;
  f.y_prev = 0.0f;
}

float washout_update(WashoutFilter& f, float x) {
  float y = -f.a1 * f.y_prev
            + f.b0 * x
            + f.b1 * f.x_prev;

  f.x_prev = x;
  f.y_prev = y;

  return y;
}

/* ---------------------------------------------------------------------------
 *  DREF → X-PLANE
 * -------------------------------------------------------------------------*/
bool sendDREF(const char* dref, float value) {
  bufDREF[0] = 'D';
  bufDREF[1] = 'R';
  bufDREF[2] = 'E';
  bufDREF[3] = 'F';
  bufDREF[4] = 0;

  uint32_t u;
  memcpy(&u, &value, sizeof(value));
  bufDREF[5] = (uint8_t)(u & 0xFF);
  bufDREF[6] = (uint8_t)((u >> 8) & 0xFF);
  bufDREF[7] = (uint8_t)((u >> 16) & 0xFF);
  bufDREF[8] = (uint8_t)((u >> 24) & 0xFF);

  size_t dlen = strnlen(dref, 499);
  memcpy(&bufDREF[9], dref, dlen);
  bufDREF[9 + dlen] = 0;

  udp.beginPacket(XPLANE_IP, XPLANE_PORT);
  size_t written = udp.write(bufDREF, sizeof(bufDREF));
  bool ok = (udp.endPacket() == 1);
  return ok && (written == sizeof(bufDREF));
}

/* ---------------------------------------------------------------------------
 *  COMANDOS PARA O X-PLANE
 * -------------------------------------------------------------------------*/
void setControls(float elv_cmd_, float ail_cmd_, float rud_cmd_, float thr_cmd_) {
  elv_cmd_ = constrain(elv_cmd_, -1.0f, 1.0f);
  ail_cmd_ = constrain(ail_cmd_, -1.0f, 1.0f);
  rud_cmd_ = constrain(rud_cmd_, -1.0f, 1.0f);
  thr_cmd_ = constrain(thr_cmd_, 0.0f, 1.0f);

  sendDREF("sim/cockpit2/controls/yoke_pitch_ratio",        elv_cmd_);
  sendDREF("sim/cockpit2/controls/yoke_roll_ratio",         ail_cmd_);
  sendDREF("sim/cockpit2/controls/yoke_heading_ratio",      rud_cmd_);
  sendDREF("sim/cockpit2/engine/actuators/throttle_ratio_all", thr_cmd_);
}

/* ---------------------------------------------------------------------------
 *  DEBUG
 * -------------------------------------------------------------------------*/
void printData() {
  Serial.print(F(" roll: "));
  Serial.print(roll_deg);
  Serial.print(F(" pitch: "));
  Serial.print(pitch_deg);
  Serial.print(F(" yaw: "));
  Serial.print(hdg_mag);

  Serial.print(F(" | P: "));  Serial.print(p_deg);
  Serial.print(F(" Q: "));    Serial.print(q_deg);
  Serial.print(F(" R: "));    Serial.print(r_deg);

  Serial.print(F(" | g_n: ")); Serial.print(n_g);
  Serial.print(F(" g_a: "));   Serial.print(a_g);
  Serial.print(F(" g_s: "));   Serial.print(s_g);

  Serial.print(F(" | lat: ")); Serial.print(lat);
  Serial.print(F(" lon: "));   Serial.print(lon);
  Serial.print(F(" alt: "));   Serial.print(alt);
  Serial.print(F(" vel: "));   Serial.print(vel);

  Serial.print(F(" | ele_cmd: ")); Serial.print(elv_cmd);
  Serial.print(F(" ail_cmd: "));   Serial.print(ail_cmd);
  Serial.print(F(" rud_cmd: "));   Serial.print(rud_cmd);
  Serial.print(F(" thr_cmd: "));   Serial.print(thr_cmd);

  Serial.print(F(" | REF_ALT: "));          Serial.print(REF_ALT);
  Serial.print(F(" REF_THETA[deg]: "));     Serial.print(REF_THETA / DEG2RAD);
  Serial.print(F(" REF_VEL: "));            Serial.print(REF_VEL);
  Serial.print(F(" REF_ROLL[deg]: "));      Serial.print(REF_ROLL / DEG2RAD);
  Serial.print(F(" REF_YAW[deg]: "));       Serial.println(REF_YAW);
}

/* ---------------------------------------------------------------------------
 *  PROCESSAR PACOTE DATA DO X-PLANE
 * -------------------------------------------------------------------------*/
void processIncoming() {
  int pkt = udp.parsePacket();
  if (pkt <= 0) return;

  int n = udp.read(buf, sizeof(buf));
  if (n < 5) return;

  if (!(buf[0] == 'D' && buf[1] == 'A' && buf[2] == 'T' && buf[3] == 'A')) {
    Serial.println(F("Pacote sem cabecalho DATA\\0"));
    return;
  }

  for (int i = 5; i + 36 <= n; i += 36) {
    int idx = get_le32(&buf[i]);
    float v[8];
    for (int k = 0; k < 8; ++k) {
      v[k] = get_le_f32(&buf[i + 4 + 4 * k]);
    }

    switch (idx) {
      case RPY:
        pitch_deg = v[0];
        roll_deg  = v[1];
        hdg_mag   = v[2];
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
        alt = v[2];   // altitude MSL
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
}

/* ---------------------------------------------------------------------------
 *  CONTROLADORES
 * -------------------------------------------------------------------------*/
void longControl() {
  float pitch = pitch_deg * DEG2RAD;
  float q     = q_deg     * DEG2RAD;

  thr_cmd = pi_update(pi_vel, REF_VEL, vel, 0.0f);

  theta_ref_from_alt = pi_update(pi_alt, REF_ALT, alt, 0.0f) + REF_THETA;

  elv_cmd = pi_update(pi_theta, theta_ref_from_alt, pitch, q);
}

void lateroControl() {
  float roll = roll_deg * DEG2RAD;
  float yaw  = hdg_mag  * DEG2RAD;

  float p = p_deg * DEG2RAD;
  float r = r_deg * DEG2RAD;

  float r_wash = washout_update(washout_yaw, r);

  float ref_yaw_rad = REF_YAW * DEG2RAD;
  rud_cmd = pi_update(pi_psi, 0.0f, 0.0f, r_wash);

  float ref_roll = 0.015f * (ref_yaw_rad - yaw) - roll;
  ail_cmd = pi_update(pi_phi, ref_roll, roll, p);
}

void longRollControl() {
  float roll  = roll_deg  * DEG2RAD;
  float pitch = pitch_deg * DEG2RAD;
  float yaw   = hdg_mag   * DEG2RAD;

  float p = p_deg * DEG2RAD;
  float q = q_deg * DEG2RAD;
  float r = r_deg * DEG2RAD;

  thr_cmd = pi_update(pi_vel, REF_VEL, vel, 0.0f);

  float theta_ref_from_alt_loc = pi_update(pi_alt, REF_ALT, alt, 0.0f) + REF_THETA;

  elv_cmd = pi_update(pi_theta, theta_ref_from_alt_loc, pitch, q);
  ail_cmd = pi_update(pi_phi, REF_ROLL, roll, p);

  float ref_yaw_rad = REF_YAW * DEG2RAD;
  rud_cmd = pi_update(pi_psi, ref_yaw_rad, yaw, r);
}

/* ---------------------------------------------------------------------------
 *  DEBUG WRAPPER
 * -------------------------------------------------------------------------*/
unsigned long last_monitor_time = 0;

void debug(unsigned long now) {
  if (now - last_monitor_time >= 1000) {
#if DEBUG_DATA
    printData();
#endif
    rpy_hz = pqr_hz = gload_hz = gps_hz = cmd_hz = thr_hz = vel_hz = 0;
    last_monitor_time = now;
  }
}

/* ---------------------------------------------------------------------------
 *  SETUP (ESP32 + WiFi)
 * -------------------------------------------------------------------------*/
unsigned long last_control_time = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Conectando ao WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");

  Serial.print("IP local ESP32: ");
  Serial.println(WiFi.localIP());

  if (udp.begin(LISTEN_PORT)) {
    Serial.print("UDP ouvindo na porta ");
    Serial.println(LISTEN_PORT);
  } else {
    Serial.println("Falha ao iniciar UDP");
  }

  float Ts = 0.01f;  // 100 Hz
  float Tw = 5.0f;   // washout ~5s
  washout_init(washout_yaw, Tw, Ts);

  last_control_time = millis();
}

/* ---------------------------------------------------------------------------
 *  LOOP PRINCIPAL
 * -------------------------------------------------------------------------*/
void loop() {
  processIncoming();

  unsigned long now = millis();
  if (now - last_control_time >= 10) {
#ifdef LONG_LATERO
    longControl();
    lateroControl();
#endif

#ifdef LONG_ROLL_CONTROL
    longRollControl();
#endif

    setControls(elv_cmd, ail_cmd, rud_cmd, thr_cmd);

    last_control_time = now;
  }

  debug(now);

  delay(1);
}
