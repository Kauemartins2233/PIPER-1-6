/*
 * arduino_guiagem_controle.ino
 *
 * Arduino Mega 2560 — controle (PIDs+SAS) + guiagem (LOS waypoints) embarcados.
 * Estende arduino_controlador_manual.ino com:
 *   - storage de waypoints recebidos pela serial (1 upload por missao)
 *   - guiagem LOS (atan2(dE,dN)) com switch por raio de aceitacao
 *   - heading hold (psi -> phi_ref) com ganho 0.8 (igual NL_guidance.slx)
 *
 * Protocolo (UART0 @ 115200):
 *   PC -> MEGA  WP_UPLOAD : [0xBB 0x66] uint8 N + N*(4 floats LE)
 *                            WP = [N, E, alt, V]   max N=32
 *   MEGA -> PC  WP_ACK    : [0x66 0xBB] uint8 N
 *
 *   PC -> MEGA  SENSORS   : [0xAA 0x55] + 15 floats LE   (62 B)
 *     sensors = [p q r u v w phi theta psi VT alpha beta xN xE alt]
 *                                                       ^^^^^ (antes 0 0)
 *   MEGA -> PC  ACTUATORS : [0x55 0xAA] + 4 floats LE    (18 B)
 *     U = [thr elev ail rud]
 */

#include <Arduino.h>
#include <string.h>
#include <math.h>

// ----------------- Parametros base -----------------
static const float TS         = 0.01f;
static const float THETA_EQ   = -0.121684f;
static const float UE_THR     = 0.491387f;
static const float UE_ELEV    = 0.015456f;

static const float KQ         = 0.1f;
static const float KP_SAS     = 0.119f;
static const float KR         = 0.15f;
static const float K_PSI       = 0.3f;     // heading hold (Gain1 do NL_guidance.slx)
static const float PSI_ERR_SAT = 1.0f;     // Saturation rad antes do gain
static const float TF_TAU_PSI  = 1.0f;     // Transfer Fcn 1/(s+1) em psi_ref
static float psi_ref_filt      = 0.0f;     // estado do filtro (init no 1o passo)
static bool  psi_filt_init     = false;

static const float ELEV_LIM   = 0.4363f;
static const float AIL_LIM    = 0.4363f;
static const float RUD_LIM    = 0.4363f;

static const float THR_ERR_LO = -0.49f;
static const float THR_ERR_HI =  0.51f;
static const float TH_REF_LO  = -0.17f;
static const float TH_REF_HI  =  0.26f;
static const float PHI_REF_LIM = 0.4363f;

// ----------------- Waypoints -----------------
#define MAX_WPS 32
static float WPs[MAX_WPS][4];     // [N, E, alt, V]
static uint8_t n_wps = 0;
static uint8_t wp_idx = 1;        // proximo WP alvo (0 = partida)
static const float R_ACCEPT = 80.0f;

// fallback (sem waypoints carregados)
static const float H_REF_DEF  = 100.0f;
static const float VT_REF_DEF = 15.1117f;

// ----------------- PID Tustin + back-calc anti-windup -----------------
typedef struct {
  float Kp, Ki, Kd, N, Kb;
  float alpha, beta;
  float I, d_prev, e_prev;
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
  float d = p->alpha * p->d_prev + p->beta * (e - p->e_prev);
  float u_raw = p->Kp * e + p->I + d;
  float u_sat = clampf(u_raw, lo, hi);
  p->I += 0.5f * p->Ki * Ts * (e + p->e_prev) + p->Kb * Ts * (u_sat - u_raw);
  p->d_prev = d;
  p->e_prev = e;
  return u_sat;
}

static PID pid_vel = { 0.05f, 0.02f,  0.01f, 20.0f, 0.5f, 0,0,0,0,0 };
static PID pid_alt = { 0.08f, 0.02f,  0.0f,  20.0f, 0.3f, 0,0,0,0,0 };
static PID pid_th  = { 0.26f, 0.143f, 0.0f,  20.0f, 0.6f, 0,0,0,0,0 };
static PID pid_phi = { 10.0f, 0.0f,   0.0f,  20.0f, 0.0f, 0,0,0,0,0 };

static float wash_r = 0.0f;

// ----------------- Guiagem LOS -----------------
static void guidance_step(float xN, float xE,
                          float* h_ref, float* V_ref, float* psi_ref) {
  if (n_wps < 2) {
    *h_ref   = H_REF_DEF;
    *V_ref   = VT_REF_DEF;
    *psi_ref = 0.0f;
    return;
  }
  if (wp_idx >= n_wps) wp_idx = n_wps - 1;

  float dN = WPs[wp_idx][0] - xN;
  float dE = WPs[wp_idx][1] - xE;
  float dist = sqrtf(dN*dN + dE*dE);

  if (dist <= R_ACCEPT && wp_idx < (uint8_t)(n_wps - 1)) {
    wp_idx++;
    dN = WPs[wp_idx][0] - xN;
    dE = WPs[wp_idx][1] - xE;
  }

  *h_ref   = WPs[wp_idx][2];
  *V_ref   = WPs[wp_idx][3];
  *psi_ref = atan2f(dE, dN);
}

static inline float wrap_pi(float a) {
  while (a >  (float)M_PI) a -= 2.0f*(float)M_PI;
  while (a < -(float)M_PI) a += 2.0f*(float)M_PI;
  return a;
}

// ----------------- Serial bufferizada -----------------
// Buffer maior pra acomodar WP_UPLOAD: 3 + 16*32 = 515 B  -> usamos 540
static uint8_t rxbuf[540];
static uint16_t rxlen = 0;

static void drain_serial(void) {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    if (rxlen < sizeof(rxbuf)) {
      rxbuf[rxlen++] = b;
    } else {
      memmove(rxbuf, rxbuf + 1, sizeof(rxbuf) - 1);
      rxbuf[sizeof(rxbuf) - 1] = b;
    }
  }
}

// Procura header em qq lugar do buffer; consome tudo antes do header.
// Retorna:
//   1 = SENSORS pronto, copia 15 floats em out_sensors
//   2 = WP_UPLOAD pronto, popula WPs[][] e envia ACK
//   0 = nada ainda
static uint8_t try_parse_frame(float out_sensors[15]) {
  for (uint16_t i = 0; i + 1 < rxlen; i++) {
    // SENSORS: [0xAA 0x55] + 60 B
    if (rxbuf[i] == 0xAA && rxbuf[i+1] == 0x55) {
      if ((uint16_t)(i + 62) > rxlen) return 0;  // espera mais bytes
      memcpy(out_sensors, &rxbuf[i + 2], 60);
      uint16_t consumed = i + 62;
      uint16_t remain = rxlen - consumed;
      memmove(rxbuf, rxbuf + consumed, remain);
      rxlen = remain;
      return 1;
    }
    // WP_UPLOAD: [0xBB 0x66] + uint8 N + 16*N
    if (rxbuf[i] == 0xBB && rxbuf[i+1] == 0x66) {
      if ((uint16_t)(i + 3) > rxlen) return 0;
      uint8_t N = rxbuf[i + 2];
      if (N == 0 || N > MAX_WPS) {
        // descarta header invalido
        uint16_t consumed = i + 3;
        memmove(rxbuf, rxbuf + consumed, rxlen - consumed);
        rxlen -= consumed;
        return 0;
      }
      uint16_t need = 3 + (uint16_t)N * 16;
      if ((uint16_t)(i + need) > rxlen) return 0;  // espera mais
      memcpy(WPs, &rxbuf[i + 3], (uint16_t)N * 16);
      n_wps = N;
      wp_idx = 1;
      // reseta integradores das malhas pra evitar wind-up de missoes anteriores
      pid_init(&pid_vel, TS);
      pid_init(&pid_alt, TS);
      pid_init(&pid_th,  TS);
      pid_init(&pid_phi, TS);
      wash_r = 0.0f;
      // ACK
      uint8_t ack[3] = { 0x66, 0xBB, N };
      Serial.write(ack, 3);
      uint16_t consumed = i + need;
      uint16_t remain = rxlen - consumed;
      memmove(rxbuf, rxbuf + consumed, remain);
      rxlen = remain;
      return 2;
    }
  }
  // sem header conhecido: se buffer encheu, descarta um byte
  if (rxlen >= sizeof(rxbuf) - 1) {
    memmove(rxbuf, rxbuf + 1, rxlen - 1);
    rxlen--;
  }
  return 0;
}

static void send_actuators(const float U[4]) {
  uint8_t hdr[2] = { 0x55, 0xAA };
  Serial.write(hdr, 2);
  Serial.write((const uint8_t*)U, 16);
}

// ----------------- Setup/Loop -----------------
void setup(void) {
  Serial.begin(115200);
  delay(50);
  pid_init(&pid_vel, TS);
  pid_init(&pid_alt, TS);
  pid_init(&pid_th,  TS);
  pid_init(&pid_phi, TS);
  n_wps = 0;
  wp_idx = 1;
}

void loop(void) {
  drain_serial();

  float s[15];
  uint8_t kind = try_parse_frame(s);
  if (kind != 1) return;   // nada a fazer (ou era WP_UPLOAD, ja tratado)

  // mapeamento (PC envia exatamente nesta ordem)
  float p     = s[0];
  float q     = s[1];
  float r     = s[2];
  // s[3..5] = u v w  (nao usados)
  float phi   = s[6];
  float theta = s[7];
  float psi   = s[8];
  float VT    = s[9];
  // s[10..11] = alpha beta (nao usados)
  float xN    = s[12];
  float xE    = s[13];
  float alt   = s[14];

  // ---- Guiagem ----
  float h_ref, V_ref, psi_ref;
  guidance_step(xN, xE, &h_ref, &V_ref, &psi_ref);

  // ---- Longitudinal: throttle ----
  float e_vel = V_ref - VT;
  float u_vel = pid_step_tustin(&pid_vel, e_vel, TS, THR_ERR_LO, THR_ERR_HI);
  float thr_out = clampf(u_vel + UE_THR, 0.0f, 1.0f);

  // ---- Longitudinal: elevator ----
  float e_alt = h_ref - alt;
  float theta_ref = pid_step_tustin(&pid_alt, e_alt, TS, TH_REF_LO, TH_REF_HI) + THETA_EQ;

  float e_th = theta_ref - theta;
  float u_th = pid_step_tustin(&pid_th, e_th, TS, -ELEV_LIM, ELEV_LIM);
  float elev_out = clampf(u_th - KQ * q + UE_ELEV, -ELEV_LIM, ELEV_LIM);

  // ---- Lateral: heading hold psi -> phi_ref (idem NL_guidance.slx) ----
  // 1) Filtro 1/(s+1) em psi_ref (backward Euler):
  //      y[k] = (y[k-1] + Ts*u[k]) / (1 + Ts/tau)
  if (!psi_filt_init) {
    psi_ref_filt = psi_ref;          // evita transiente do zero ate atan2
    psi_filt_init = true;
  }
  psi_ref_filt = (psi_ref_filt + (TS / TF_TAU_PSI) * psi_ref) / (1.0f + TS / TF_TAU_PSI);
  // 2) erro_psi com wrap, depois saturado em ±1 rad
  float e_psi = wrap_pi(psi_ref_filt - psi);
  e_psi = clampf(e_psi, -PSI_ERR_SAT, PSI_ERR_SAT);
  // 3) Gain 0.3 -> phi_ref (limit ainda aplicado por garantia)
  float phi_ref = clampf(K_PSI * e_psi, -PHI_REF_LIM, PHI_REF_LIM);

  // ---- Lateral: aileron ----
  float e_phi = phi_ref - phi;
  float u_phi = pid_step_tustin(&pid_phi, e_phi, TS, -AIL_LIM, AIL_LIM);
  float ail_out = clampf(u_phi - KP_SAS * p, -AIL_LIM, AIL_LIM);

  // ---- Lateral: rudder (washout em r, tau = 1 s) ----
  wash_r = (wash_r + r * TS) / (1.0f + TS);
  float hp_r = r - wash_r;
  float rud_out = clampf(-KR * hp_r, -RUD_LIM, RUD_LIM);

  float U[4] = { thr_out, elev_out, ail_out, rud_out };
  send_actuators(U);
}
