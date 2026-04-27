/*
 * arduino_controlador_manual.ino
 *
 * Controlador Piper 1/6 escrito manualmente em C float32 para Arduino Mega 2560.
 * Substitui o modelo gerado pelo Simulink Coder (que usava double + RK4 e era lento
 * demais para rodar a 100 Hz no AVR).
 *
 * Protocolo serial (UART0 @ 115200):
 *   RX (host -> Mega): [0xAA 0x55] + 15 floats (little-endian) = 62 bytes
 *     sensors = [p q r u v w phi theta psi VT alpha beta 0 0 alt]
 *   TX (Mega -> host): [0x55 0xAA] + 4  floats = 18 bytes
 *     U = [thr elev ail rud]
 *
 * Malha (cascata):
 *   Throttle: PID(VT_ref - VT) -> saturacao -> + Ue_thr
 *   Elevator: PID_alt(h_ref - h) -> theta_ref;
 *             PID_th(theta_ref + theta_eq - theta) -> sat -> -Kq*q + Ue_elev
 *   Aileron : PID_phi(0 - phi) -> sat -> -Kp_sas*p
 *   Rudder  : -Kr * highpass(r)   (washout tau = 1 s)
 */

#include <Arduino.h>
#include <string.h>

// ----------------- Parametros (bate com inicializar.m) -----------------
static const float TS        = 0.01f;       // 100 Hz
static const uint32_t TS_US  = 10000UL;

static const float H_REF     = 100.0f;
static const float VT_REF    = 15.1117f;
static const float THETA_EQ  = -0.121684f;
static const float UE_THR    = 0.491387f;
static const float UE_ELEV   = 0.015456f;

static const float KQ        = 0.1f;        // SAS pitch rate
static const float KP_SAS    = 0.119f;      // SAS roll rate
static const float KR        = 0.15f;       // SAS yaw rate (apos washout)

// limites dos atuadores (rad e fracao throttle)
static const float ELEV_LIM  = 0.4363f;
static const float AIL_LIM   = 0.4363f;
static const float RUD_LIM   = 0.4363f;

// saturacoes internas (erros de PID, iguais ao Simulink)
static const float THR_ERR_LO = -0.49f;
static const float THR_ERR_HI =  0.51f;
static const float TH_REF_LO  = -0.17f;
static const float TH_REF_HI  =  0.26f;

// ----------------- PID discreto com derivada filtrada -----------------
// PID discretizado por Tustin (bilinear), com anti-windup back-calculation.
// Integrador trapezoidal:  I[k] = I[k-1] + Ki*Ts/2*(e[k]+e[k-1])
// Derivada filtrada:       d[k] = alpha*d[k-1] + beta*(e[k]-e[k-1])
//   alpha = (2 - N*Ts)/(2 + N*Ts);  beta = 2*N*Kd/(2 + N*Ts)
// Anti-windup: I += Kb*Ts*(u_sat - u_raw)  (back-calculation, igual ao Simulink)
typedef struct {
  float Kp, Ki, Kd, N, Kb;
  float alpha, beta;     // pre-calculados em pid_init
  float I;               // integrador
  float d_prev;          // derivada filtrada anterior
  float e_prev;          // erro anterior
} PID;

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void pid_init(PID* p, float Ts) {
  float NTs = p->N * Ts;
  p->alpha = (2.0f - NTs) / (2.0f + NTs);
  p->beta  = 2.0f * p->N * p->Kd / (2.0f + NTs);
  p->I = 0.0f; p->d_prev = 0.0f; p->e_prev = 0.0f;
}

static float pid_step_tustin(PID* p, float e, float Ts, float lo, float hi) {
  // Derivada filtrada (Tustin)
  float d = p->alpha * p->d_prev + p->beta * (e - p->e_prev);

  // Saida pre-saturacao
  float u_raw = p->Kp * e + p->I + d;
  float u_sat = clampf(u_raw, lo, hi);

  // Integrador trapezoidal + back-calc anti-windup
  p->I += 0.5f * p->Ki * Ts * (e + p->e_prev) + p->Kb * Ts * (u_sat - u_raw);

  p->d_prev = d;
  p->e_prev = e;
  return u_sat;
}

// Instancias com os ganhos do inicializar.m (para comparacao com discretizacao do Marcelo)
static PID pid_vel = { 0.05f, 0.02f,  0.01f, 20.0f, 0.5f, 0,0,0,0,0 };
static PID pid_alt = { 0.08f, 0.02f,  0.0f,  20.0f, 0.3f, 0,0,0,0,0 };
static PID pid_th  = { 0.26f, 0.143f, 0.0f,  20.0f, 0.6f, 0,0,0,0,0 };
static PID pid_phi = { 10.0f, 0.0f,   0.0f,  20.0f, 0.0f, 0,0,0,0,0 };

// Washout (low-pass state de r; saida highpass = r - wash_r)
static float wash_r = 0.0f;

// ----------------- Serial bufferizada -----------------
static uint8_t rxbuf[80];
static uint8_t rxlen = 0;

static void drain_serial(void) {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    if (rxlen < sizeof(rxbuf)) {
      rxbuf[rxlen++] = b;
    } else {
      // rola: descarta mais antigo
      memmove(rxbuf, rxbuf + 1, sizeof(rxbuf) - 1);
      rxbuf[sizeof(rxbuf) - 1] = b;
    }
  }
}

// Tenta achar um frame [0xAA 0x55] + 60 bytes no buffer.
// Retorna true se achou, copia os 15 floats em 'out', e consome o frame.
static bool try_parse_frame(float out[15]) {
  if (rxlen < 62) return false;
  for (uint8_t i = 0; (uint8_t)(i + 62) <= rxlen; i++) {
    if (rxbuf[i] == 0xAA && rxbuf[i + 1] == 0x55) {
      memcpy(out, &rxbuf[i + 2], 60);
      uint8_t consumed = i + 62;
      uint8_t remain = rxlen - consumed;
      memmove(rxbuf, rxbuf + consumed, remain);
      rxlen = remain;
      return true;
    }
  }
  // sem header: se buffer quase cheio, joga tudo fora
  if (rxlen > 70) rxlen = 0;
  return false;
}

static void send_actuators(const float U[4]) {
  uint8_t hdr[2] = { 0x55, 0xAA };
  Serial.write(hdr, 2);
  Serial.write((const uint8_t*)U, 16);
}

// ----------------- Setup/Loop -----------------
void setup(void) {
  Serial.begin(115200);
  // pequena pausa pra estabilizar; host ja espera 3 s apos abrir porta
  delay(50);
  pid_init(&pid_vel, TS);
  pid_init(&pid_alt, TS);
  pid_init(&pid_th,  TS);
  pid_init(&pid_phi, TS);
}

void loop(void) {
  drain_serial();

  float s[15];
  if (!try_parse_frame(s)) return;   // nada a fazer ate proximo frame

  // mapeamento (igual ao host em inicializar_HIL_simulink.m):
  float p   = s[0];
  float q   = s[1];
  float r   = s[2];
  // s[3..5] = u v w  (nao usados)
  float phi   = s[6];
  float theta = s[7];
  // s[8] = psi (nao usado, wings-level)
  float VT    = s[9];
  // s[10..11] = alpha beta (nao usados)
  // s[12..13] = 0 0
  float alt   = s[14];

  // ---- Longitudinal: throttle ----
  float e_vel = VT_REF - VT;
  float u_vel = pid_step_tustin(&pid_vel, e_vel, TS, THR_ERR_LO, THR_ERR_HI);
  float thr_out = clampf(u_vel + UE_THR, 0.0f, 1.0f);

  // ---- Longitudinal: elevator ----
  float e_alt = H_REF - alt;
  float theta_ref = pid_step_tustin(&pid_alt, e_alt, TS, TH_REF_LO, TH_REF_HI) + THETA_EQ;

  float e_th = theta_ref - theta;
  float u_th = pid_step_tustin(&pid_th, e_th, TS, -ELEV_LIM, ELEV_LIM);
  float elev_out = clampf(u_th - KQ * q + UE_ELEV, -ELEV_LIM, ELEV_LIM);

  // ---- Lateral: aileron (wings level) ----
  float e_phi = 0.0f - phi;
  float u_phi = pid_step_tustin(&pid_phi, e_phi, TS, -AIL_LIM, AIL_LIM);
  float ail_out = clampf(u_phi - KP_SAS * p, -AIL_LIM, AIL_LIM);
  ail_out = clampf(ail_out, -AIL_LIM, AIL_LIM);

  // ---- Lateral: rudder (washout em r, tau = 1 s) ----
  // lp_dot = -lp + r  ->  backward Euler: lp = (lp + r*TS) / (1 + TS)
  wash_r = (wash_r + r * TS) / (1.0f + TS);
  float hp_r = r - wash_r;
  float rud_out = clampf(-KR * hp_r, -RUD_LIM, RUD_LIM);

  float U[4] = { thr_out, elev_out, ail_out, rud_out };
  send_actuators(U);
}
