// XPlaneData.h
#pragma once // Garante que o arquivo só seja incluído uma vez por compilação

#include <WiFiUdp.h>

// Classe que captura e armazena dados de telemetria enviados pelo X-Plane
class XPlaneData {
public:
  // Enum com índices dos blocos de dados no protocolo DATA do X-Plane.
  // Cada índice corresponde a um tópico específico:
  enum Idx {
    SPEED = 3,          // Velocidades
    G_LOAD = 4,         // Aceleração G (normal, axial, lateral)
    CMD = 8,            // Comandos de controle (profundor, aileron, leme)
    CMD_TRIM = 8,       // Trim
    RPY = 18,           // Roll, Pitch, Yaw (em graus)
    PQR = 17,           // Taxas angulares (p, q, r) em deg/s
    GPS = 20,           // Latitude, Longitude, Altitude
    THROTTLE = 26,      // Posição atual do throttle
  };

  // Atitude (em graus)
  float pitch_deg, roll_deg, yaw_deg, rpy_hz;

  // Taxas angulares (em deg/s)
  float p_deg, q_deg, r_deg, pqr_hz;

  // Aceleração G
  float norml_g, axial_g, side_g, gload_hz;

  // Posição geográfica (GPS)
  float lat, lon, alt, gps_hz;

  // Comandos de entrada (elevator, aileron, rudder)
  float ailrn, elev, ruddr, cmd_hz;

  // Throttle atual
  float thro, thro_hz;

  // Throttle atual
  float vel, vel_hz;
  
  // Inicializa o socket UDP para escutar pacotes DATA vindos do X-Plane
  void begin(int port);

  // leitura dos pacotes
  void processIncoming();

private:
  // Instância UDP para receber o stream DATA
  WiFiUDP udp;

  // Função interna para interpretar um único pacote DATA recebido
  void parseDataPacket(uint8_t* buf, int len);
};
