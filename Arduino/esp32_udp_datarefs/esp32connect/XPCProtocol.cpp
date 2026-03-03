// XPCProtocol.cpp
#include <Arduino.h>
#include <WiFi.h>
#include "XPCProtocol.h"

// Helpers para converter números no formato Little Endian (LE)
// Esse formato é usado nos pacotes enviados e recebidos via UDP pelo XPlaneConnect

// Converte 4 bytes LE em um int32
static inline int32_t le32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] |
                  ((uint32_t)p[1] << 8) |
                  ((uint32_t)p[2] << 16) |
                  ((uint32_t)p[3] << 24));
}

// Converte 4 bytes LE em um float IEEE 754
static inline float leF32(const uint8_t* p) {
  uint32_t u = (uint32_t)p[0] |
               ((uint32_t)p[1] << 8) |
               ((uint32_t)p[2] << 16) |
               ((uint32_t)p[3] << 24);
  float f;
  memcpy(&f, &u, sizeof(f));
  return f;
}

// Inicializa o protocolo XPlaneConnect
void XPCProtocol::begin(const char* ip, uint16_t port) {
  xpcPort = port;
  // Converte string IP para tipo IPAddress
  xpcIP.fromString(ip);

  // Inicia um socket UDP em uma porta aleatória (0 = automática)
  // Serve exclusivamente para se comunicar com o plugin XPlaneConnect
  udp.begin(0);
}

// Envia requisição GETD com uma lista de DataRefs
void XPCProtocol::sendGETD(const std::vector<const char*>& datarefs) {
  // Guarda a lista interna para reutilizar depois
  refs.clear();
  refs.reserve(datarefs.size());
  for (auto r : datarefs) {
    refs.emplace_back(String(r)); // Armazena como String
  }
  sendGETD();
}

// Envia a última requisição GETD salva
void XPCProtocol::sendGETD() {
  if (refs.empty()) return;  // Nenhum DataRef para requisitar

  uint8_t buf[256];  // Buffer para montar pacote GETD
  int offset = 0;
  uint8_t count = refs.size();  // Número de DataRefs

  // Cabeçalho "GETD"
  buf[offset++] = 'G';
  buf[offset++] = 'E';
  buf[offset++] = 'T';
  buf[offset++] = 'D';

  // 1 byte de padding (sempre 0)
  buf[offset++] = 0x00;

  // Número de DataRefs
  buf[offset++] = count;

  // Para cada DataRef:
  // [comprimento][string ref]
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t len = (uint8_t)refs[i].length();  // Tamanho do nome
    buf[offset++] = len;
    memcpy(&buf[offset], refs[i].c_str(), len); // Copia a string (sem null)
    offset += len;
  }

  // Envia pacote UDP ao XPlaneConnect
  udp.beginPacket(xpcIP, xpcPort);
  udp.write(buf, offset);
  udp.endPacket();
}

// Lê e interpreta respostas RESP do XPlaneConnect
void XPCProtocol::handleResponses() {
  // Verifica se chegou algum pacote UDP
  int pkt = udp.parsePacket();
  if (pkt <= 0) return;

  uint8_t buf[512];
  int n = udp.read(buf, sizeof(buf)); // Lê todo o pacote
  if (n < 4) return;

  // Verifica cabeçalho "RESP" (padrão do XPlaneConnect)
  if (buf[0]=='R' && buf[1]=='E' && buf[2]=='S' && buf[3]=='P') {
    parseRESP(buf, n);
    acc_hz++;
  } 
}

// Faz o parse do pacote RESP retornado pelo GETD
void XPCProtocol::parseRESP(uint8_t* buf, int n) {
  if (n < 6) return; // pacote muito pequeno

  uint8_t count = buf[5]; // Número de DataRefs retornados
  int offset = 6;         // Começa após header + padding + count

  // Armazenar os 3 primeiros valores (ax, ay, az)
  float vals[3] = {0.0f, 0.0f, 0.0f};

  for (uint8_t i = 0; i < count && i < 3; ++i) {
    if (offset >= n) break;

    uint8_t rowLen = buf[offset++]; 
    if (offset + rowLen * 4 > n) break;

    // Pega apenas o primeiro valor de cada DataRef (escalares)
    float v0 = leF32(&buf[offset]);
    offset += rowLen * 4;

    vals[i] = v0;
  }

  // Atribui valores decodificados
  // Ordem assumida: ax, ay, az
  local_ax = vals[0];
  local_ay = vals[1];
  local_az = vals[2];

  // Para debugging
  // Serial.printf("Ax=%.3f Ay=%.3f Az=%.3f\n", local_ax, local_ay, local_az);
}
