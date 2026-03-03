#include "XPlaneData.h"

// Helpers para interpretar valores little-endian (X-Plane envia os pacotes como LE)

// Converte 4 bytes a partir de um ponteiro para um inteiro de 32 bits (signed)
static inline int32_t le32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

// Converte 4 bytes little-endian para um float IEEE 754
static inline float leF32(const uint8_t* p) {
  uint32_t u = le32(p);        
  float f;
  memcpy(&f, &u, sizeof(f)); 
  return f;
}

// Inicializa o socket UDP para começar a escutar pacotes enviados pelo X-Plane
void XPlaneData::begin(int port) {
  udp.begin(port);   // Escuta pacotes UDP na porta especificada
}

// Processa os pacotes disponíveis no socket
void XPlaneData::processIncoming() {
  int pkt = udp.parsePacket();  // verifica se chegou algum pacote
  if (pkt <= 0) return;         // se não chegou nada finaliza a função

  uint8_t buf[300];             // buffer para armazenar os bytes recebidos
  int n = udp.read(buf, sizeof(buf));  // lê o pacote
  if (n < 5) return;            

  // Valida que o pacote é do tipo "DATA" (telemetria padrão)
  if (buf[0] == 'D' && buf[1] == 'A' && buf[2] == 'T' && buf[3] == 'A') {
    parseDataPacket(buf, n);    // envia para o parser de pacotes
  }
}

// Faz o parsing de um pacote DATA completo
// Estrutura: [ "D","A","T","A",padding, idx(4), f1(4), f2(4)...f8(4) ] por bloco
void XPlaneData::parseDataPacket(uint8_t* buf, int len) {
  // Cada bloco tem 36 bytes: 4 do índice + 8 floats (4 bytes cada)
  for (int i = 5; i + 36 <= len; i += 36) {
    int idx = le32(&buf[i]);       // índice da seção DATA

    // Lê os oito valores de telemetria
    float v[8];
    for (int k = 0; k < 8; ++k) {
      v[k] = leF32(&buf[i + 4 + 4 * k]);  // Vê cada float em LE
    }

    // Atualiza os dados conforme o índice mapeado
    if (idx == RPY) {
      pitch_deg = v[0];
      roll_deg  = v[1];
      yaw_deg   = v[2];
      rpy_hz++;                   
    } 
    else if (idx == PQR) {
      p_deg = v[1];                
      q_deg = v[0];
      r_deg = v[2];
      pqr_hz++;
    } 
    else if (idx == G_LOAD) {
      norml_g = v[4];              
      axial_g = v[5];             
      side_g  = v[6];              
      gload_hz++;
    } 
    else if (idx == GPS) {
      lat = v[0];
      lon = v[1];
      alt = v[2];
      gps_hz++;
    }
    else if (idx == CMD) {
      elev  = v[0];
      ailrn = v[1];
      ruddr = v[2];
      cmd_hz++;
    } 
    else if (idx == THROTTLE) {
      thro = v[0];
      thro_hz++;
    }
    else if (idx == SPEED) {
      vel = v[3]* 0.5;
      vel_hz++;
    }
  }
}
