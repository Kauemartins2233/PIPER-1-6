// XPlaneControl.cpp

#include <Arduino.h>
#include <WiFi.h>
#include "XPlaneControl.h"

// Método que envia um pacote DREF ao X-Plane
// DREF é o protocolo padrão usado pelo X-Plane para "setar" DataRefs remotamente.
void XPlaneControl::sendDREF(const char* dref, float value) {
  uint8_t buf[509];          // Buffer responsável por armazenar o pacote
  memset(buf, 0, sizeof(buf));  // Inicializa o buffer com zeros

  // Cabeçalho "DREF\0"
  // O X-Plane exige esse formato como pré-condição para os pacotes de controle
  buf[0] = 'D';
  buf[1] = 'R';
  buf[2] = 'E';
  buf[3] = 'F';
  buf[4] = 0;  // Padding (sempre zero)

  // Converte o valor float para um inteiro de 32 bits (IEEE754 em little-endian)
  uint32_t u;
  memcpy(&u, &value, sizeof(value));  // Copia os bytes do float para a variável u
  buf[5] =  u        & 0xFF;         // LSB
  buf[6] = (u >> 8)  & 0xFF;
  buf[7] = (u >> 16) & 0xFF;
  buf[8] = (u >> 24) & 0xFF;         // MSB

  // Copia o nome do DataRef (string) a partir da posição 9 do buffer
  size_t dlen = strnlen(dref, 499);
  memcpy(&buf[9], dref, dlen);

  // Adiciona '\0' ao fim da string do DataRef
  buf[9 + dlen] = 0;

  // Envia o pacote via UDP ao X-Plane
  udp.beginPacket(simIP, simPort);      // Prepara o envio para o IP/Porta do simulador
  udp.write(buf, sizeof(buf));          // Envia o buffer completo (509 bytes)
  udp.endPacket();                      // Finaliza o envio
}

// Inicializa o objeto com o IP e a porta usados pelo X-Plane para comandos
void XPlaneControl::begin(const char* ip, uint16_t port) {
  simPort = port;             // Armazena a porta definida (geralmente 49000)
  simIP.fromString(ip);       // Converte string IP em tipo IPAddress
}

// Envia comandos de entrada para os controles principais do X-Plane
// elv — comando de profundor [-1 a +1]
// ail — comando de aileron [-1 a +1]
// rud — comando de leme (rudder) [-1 a +1]
// thr — posição do throttle [0 a 1]
void XPlaneControl::sendCommands(float elv, float ail, float rud, float thr) {
  // Verificação para que os valores estejam dentro dos limites esperados -1 e 1
  elv = constrain(elv, -1.0f, 1.0f);
  ail = constrain(ail, -1.0f, 1.0f);
  rud = constrain(rud, -1.0f, 1.0f);
  thr = constrain(thr,  0.0f, 1.0f);

  // Manda cada comando via seu respectivo DataRef
  sendDREF("sim/cockpit2/controls/yoke_pitch_ratio",            elv);  // Profundor
  sendDREF("sim/cockpit2/controls/yoke_roll_ratio",             ail);  // Aileron
  sendDREF("sim/cockpit2/controls/yoke_heading_ratio",          rud);  // Leme
  sendDREF("sim/cockpit2/engine/actuators/throttle_ratio_all",  thr);  // Throttle
}
