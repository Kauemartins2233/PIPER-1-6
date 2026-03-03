/*
Este código realiza a interface completa entre um microcontrolador ESP32 e o simulador de voo X-Plane, 
utilizando tanto o protocolo nativo de telemetria do simulador (DATA) 
quanto o plugin XPlaneConnect para leitura de DataRefs (em específico acelerações local).
*/

#include <WiFi.h>
#include "XPlaneData.h"
#include "XPlaneControl.h"
#include "XPCProtocol.h"
#include "PID.h"
#include "constants.h"
#define DEG2RAD 0.017453293
#define RAD2DEG 57.295779513
#define LATERO
//###################################################################################################################################
// MODIFICAR AQUI COM OS DADOS DE REDE
//###################################################################################################################################
// === CONFIG ===
// Credenciais da rede Wi-Fi que o ESP32 vai usar
const char* ssid = "Galaxy A10s8881";
const char* password = "xgla9652";

// Configurações das portas IP para comunicação UDP com o X-plane
const char* SIM_IP = "192.168.231.65";  // IP da máquina rodando (No X-Plane está em Settings-->Net Connections--> Basic: YOUR IP address is:<IP_ADDRESS>)
const uint16_t DATA_PORT = 49010;       // Porta local do ESP32 para receber pacotes DATA (No X-Plane está em Settings-->Net Connections--> Advanced: IP of data receiver (from data output screen))
const uint16_t DREF_PORT = 49000;       // Porta nativa de controle do X-Plane (DREF/CMND) (No X-Plane está em Settings-->Net Connections--> UDP Port: port that we receive on
const uint16_t XPC_PORT = 49009;        // Porta usada pelo plugin XPlaneConnect (GETD/RESP) por padrão é a porta 49009
const int SERIAL_BAUD_RATE = 115200;    // define a velocidade da comunicação serial

bool DEBUG_DATA = true;  //Flag para imprimir no console os dados recebidos
bool DEBUG_HZ = false;   //Flag para imprimir no console a frequencia dos dados

// Constantes de controle
// const float Kp_vel = 0.00644f;
// const float Ki_vel = 0.0548f;
// const float Kd_vel = 0.0;
// const float vel_max_out = 1.0;
// const float vel_min_out = 0.0;

// const float Kp_alt = 0.222f;
// const float Ki_alt = 0.00146f;
// const float Kd_alt = 0;
// const float alt_max_out = 10 * DEG2RAD;
// const float alt_min_out = -10 * DEG2RAD;

// const float Kp_theta = 0.063187;
// const float Ki_theta = 0.009672;
// const float Kd_theta = 0;
// const float Kv_theta = 0.1;
// const float theta_max_out = 0.5;
// const float theta_min_out = -0.5;

// const float Kp_roll = 0.05;
// const float Ki_roll = 0.005;
// const float Kd_roll = 0.0;
// const float Kv_roll = 0.0;
// const float roll_max_out = 0.5;
// const float roll_min_out = -0.5;

// const float Kp_yaw = 0.05f;
// const float Ki_yaw = 0.005f;
// const float Kd_yaw = 0.0;
// const float Kv_yaw = 0.0;
// const float yaw_max_out = 0.5;
// const float yaw_min_out = -0.5;

struct PI_Digital {
  float b0;
  float b1;
  float e_prev;   // e[k-1]
  float u_prev;   // u[k-1]
};

float pi_update(PI_Digital &pi, float setpoint, float measurement) {
  float e = setpoint - measurement;

  float u = pi.u_prev
          + pi.b0 * e
          + pi.b1 * pi.e_prev;

  // Atualiza estados
  pi.e_prev = e;
  pi.u_prev = u;

  return u;
}

PI_Digital pi_vel   = {0.0067f,  -0.0062f, 0.0f, 0.0f};
PI_Digital pi_theta = {0.6368f,  -0.6272f, 0.0f, 0.0f};
PI_Digital pi_alt   = {0.2220f,  -0.2220f, 0.0f, 0.0f};

float ref_alt = 1500;           // referência de altitude
float ref_vel = 20.0f;          // referência de velocidade frontal
float ref_theta = -0.1217;      // referência para o theta
float ref_roll = 0.0;           // referência de roll
float ref_yaw = 100 * DEG2RAD;  // referência de yaw
//###################################################################################################################################
//###################################################################################################################################

// === Objetos globais ===
// Classe para processamento dos pacotes DATA (dados nativos do X-Plane)
XPlaneData data;

// Classe para gerenciamento do envio de comandos
XPlaneControl ctrl;

// Classe para comunicação com o plugin XPlaneConnect (via GETD e RESP)
XPCProtocol xpc;

// Controle da taxa de requisições GETD (para leitura de aceleração no corpo)
unsigned long last_getd_time = 0;
const unsigned long GETD_INTERVAL_MS = 10;  // Intervalo de ~10 ms (≈100 Hz)

unsigned long count_loop = 0;


unsigned long last_monitor_time = 0;      // tempo para monitorar o debug
unsigned long last_lon_control_time = 0;  // tempo para monitorar o controle logitudinal
unsigned long last_lat_control_time = 0;  // tempo para monitorar o controle latero direcional

// Controle PID
PID pid_vel(Kp_vel, Ki_vel, Kd_vel, 0.0, vel_min_out, vel_max_out, 'v');
PID pid_alt(Kp_alt, Ki_alt, Kd_alt, 0.0, alt_min_out, alt_max_out, 'a');
PID pid_theta(Kp_theta, Ki_theta, 0, Kv_theta, theta_min_out, theta_max_out, 't');
PID pid_roll(Kp_roll, Ki_roll, Kd_roll, Kv_roll, roll_min_out, roll_max_out, 'r');
PID pid_yaw(Kp_yaw, Ki_yaw, Kd_yaw, Kv_yaw, yaw_min_out, yaw_max_out, 'y');

float elv_cmd = 0.0f;
float ail_cmd = 0.00f;
float rud_cmd = 0.00f;
float thr_cmd = 0.00f;

// Configuração inicial do ESP32 e das comunicações
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
  Serial.println("\nConectando ao WiFi...");

  // Inicializa conexão Wi-Fi em modo estação
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Aguarda até estabelecer conexão
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(400);
  }

  // Feedback de conexão bem-sucedida
  Serial.println("\nWiFi OK!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Inicializa a leitura de pacotes DATA
  data.begin(DATA_PORT);

  // Inicializa envio de DREF e comandos ao simulador
  ctrl.begin(SIM_IP, DREF_PORT);

  // Inicializa o canal de requisições GETD via XPlaneConnect
  xpc.begin(SIM_IP, XPC_PORT);

  // Envia uma primeira requisição GETD pedindo aceleração linear nos 3 eixos do corpo
  // xpc.sendGETD({ "sim/flightmodel/position/local_ax",
  //                "sim/flightmodel/position/local_ay",
  //                "sim/flightmodel/position/local_az" });

  last_lon_control_time = millis();
  last_lat_control_time = millis();
  ctrl.sendCommands(0.0, 0.0, 0.0, 0.0);
}

void loop() {
  count_loop++;
  // 1) Faz requisições GETD periódicas para obter aceleração linear
  if (millis() - last_getd_time >= GETD_INTERVAL_MS) {
    //xpc.sendGETD();
    last_getd_time = millis();
  }

  // 2) Processa respostas do plugin XPlaneConnect
  //xpc.handleResponses();

  // 3) Processa pacotes nativos de DATA enviados pelo X-Plane
  data.processIncoming();

  // 4) Envia comandos de controle ao simulador
  //ctrl.sendCommands(0.3f, 0.0f, 0.0f, 0.6f); // sendCommands(elev, ailrn,ruddr,thro)

  //Controle logitudinal + latero direcional
  //longControl();
  //lateroControl();// TODO

  //Controle logitudinal+roll
  longRollControl();

  debug();
  // Delay para evitar travamento
  delay(1);
}

void debug() {
  // ---------- Print Loop HZ ----------
  if (millis() - last_monitor_time >= 1000) {  // verifica cada segundo
    Serial.println("-----------------------------");
    //(DEBUG)
    if (DEBUG_DATA) printState();
    if (DEBUG_HZ) printStateHz();
    count_loop = 0;
    data.rpy_hz = 0;
    data.pqr_hz = 0;
    data.gload_hz = 0;
    data.gps_hz = 0;
    data.cmd_hz = 0;
    data.thro_hz = 0;
    xpc.acc_hz = 0;
    last_monitor_time = millis();
  }
}

/***************************************************************************************
FUNÇÔES PARA DEBUG
****************************************************************************************/
// Função para imprimir todos os dados recebidos
void printState() {
  Serial.printf(
    "ax=%.3f  ay=%.3f  az=%.3f | roll=%.2f  pitch=%.2f  yaw=%.2f | "
    "p=%.3f  q=%.3f  r=%.3f | g-n=%.3f  g-a=%.3f  g-s=%.3f | "
    "lat=%.3f  lon=%.3f  alt=%.3f | vel=%.3f | elev=%.3f  ailrn=%.3f  ruddr=%.3f | thro=%.3f\n",

    // Aceleração linear lida via XPlaneConnect
    xpc.local_ax, xpc.local_ay, xpc.local_az,

    // Attitude (roll, pitch, yaw) e taxas (p, q, r)
    data.roll_deg, data.pitch_deg, data.yaw_deg,
    data.p_deg, data.q_deg, data.r_deg,

    // G-load
    data.norml_g, data.axial_g, data.side_g,

    // GPS
    data.lat, data.lon, data.alt,

    //vel
    data.vel,

    // Comandos de entrada
    //data.elev, data.ailrn, data.ruddr, data.thro);
    elv_cmd, ail_cmd, rud_cmd, thr_cmd);
}

// Função para imprimir a frequencia de todos os dados recebidos
void printStateHz() {
  Serial.printf(
    "acc=%.3f Hz| rpy=%.2f Hz | pqr=%.2f Hz | gload=%.2f Hz | gps=%.2f Hz | vel=%.2f Hz | cmd=%.2f Hz | thro=%.2f Hz\n",
    // aceleração linear lida via XPlaneConnect
    xpc.acc_hz,
    // attitude (roll, pitch, yaw)
    data.rpy_hz,
    // taxas (p, q, r)
    data.pqr_hz,
    // G-load
    data.gload_hz,
    // GPS
    data.gps_hz,
    // vel
    data.vel_hz,
    // Comandos de entrada
    data.cmd_hz, data.thro_hz);
}

void longControl() {
  if (millis() - last_lon_control_time >= 10) {  // Check every 100 hz
    float dt = (millis() - last_lon_control_time) / 1000.0;

    float roll = data.roll_deg * DEG2RAD;
    float pitch = data.pitch_deg * DEG2RAD;
    float yaw = data.yaw_deg * DEG2RAD;

    float p = data.p_deg * DEG2RAD;
    float q = data.q_deg * DEG2RAD;
    float r = data.r_deg * DEG2RAD;

    thr_cmd = pid_vel.update(ref_vel, data.vel, 0.0, dt);
    float u_theta = pid_alt.update(ref_alt, data.alt, 0.0, dt) + ref_theta;
    elv_cmd = pid_theta.update(u_theta, pitch, q, dt);

    ctrl.sendCommands(elv_cmd, ail_cmd, rud_cmd, thr_cmd);
    last_lon_control_time = millis();
  }
}

void lateroControl() {
  if (millis() - last_lat_control_time >= 10) {  // Check every 100 hz

    float dt = (millis() - last_lat_control_time) / 1000.0;

    float roll = data.roll_deg * DEG2RAD;
    float yaw = data.yaw_deg * DEG2RAD;

    float p = data.p_deg * DEG2RAD;
    float q = data.q_deg * DEG2RAD;
    float r = data.r_deg * DEG2RAD;

    float ref_lat_roll = 0.8 * (ref_yaw - yaw) - roll;

    //ail_cmd  = pid_roll.update(ref_lat_roll, roll, p, dt);
    //rud_cmd = pid_yaw.update(ref_yaw, yaw, r, dt);

    ctrl.sendCommands(elv_cmd, ail_cmd, rud_cmd, thr_cmd);
    last_lat_control_time = millis();
  }
}

void longRollControl() {
  if (millis() - last_lon_control_time >= 10) {  // Check every 100 hz
    float dt = (millis() - last_lon_control_time) / 1000.0;

    float roll = data.roll_deg * DEG2RAD;
    float pitch = data.pitch_deg * DEG2RAD;
    float yaw = data.yaw_deg * DEG2RAD;

    float p = data.p_deg * DEG2RAD;
    float q = data.q_deg * DEG2RAD;
    float r = data.r_deg * DEG2RAD;

    // thr_cmd = pid_vel.update(ref_vel, data.vel, 0.0, dt);
    // float u_theta = pid_alt.update(ref_alt, data.alt, 0.0, dt) + ref_theta;
    // elv_cmd = pid_theta.update(u_theta, pitch, q, dt);
    // ail_cmd = pid_roll.update(ref_roll, roll, p, dt);
    // rud_cmd = pid_yaw.update(ref_yaw, yaw, r, dt);

    thr_cmd  = pi_update(pi_vel,   ref_vel,   data.vel);
    float u_theta = pi_update(pi_alt,ref_alt, data.alt,) + ref_theta;
    
    elv_cmd = pid_theta.update(u_theta, pitch, q, dt);
    ail_cmd = pid_roll.update(ref_roll, roll, p, dt);
    rud_cmd = pid_yaw.update(ref_yaw, yaw, r, dt);


    ctrl.sendCommands(elv_cmd, ail_cmd, rud_cmd, thr_cmd);
    last_lon_control_time = millis();
  }
}