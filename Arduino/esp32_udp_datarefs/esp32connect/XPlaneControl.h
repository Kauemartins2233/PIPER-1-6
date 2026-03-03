// XPlaneControl.h

#pragma once  // Garante que o arquivo só seja incluído uma vez por compilação

#include <WiFiUdp.h>  // Biblioteca para comunicação UDP no ESP32

// Classe responsável por enviar comandos de controle ao simulador X-Plane
// por meio do protocolo UDP nativo
class XPlaneControl {
public:
  // Inicializa o objeto com o IP do simulador e a porta utilizada para comando
  // ip   -> endereço do computador rodando o X-Plane
  // port -> porta de comunicação nativa (geralmente 49000)
  void begin(const char* ip, uint16_t port);

  // Envia comandos diretamente para o X-Plane:
  //  elv -> posição do profundor (elevator) [-1.0 a +1.0]
  //  ail -> posição do aileron            [-1.0 a +1.0]
  //  rud -> posição do leme (rudder)      [-1.0 a +1.0]
  //  thr -> posição do throttle           [ 0.0 a +1.0]
  void sendCommands(float elv, float ail, float rud, float thr);

private:
  WiFiUDP udp;          // Socket UDP para enviar pacotes ao simulador
  IPAddress simIP;      // Endereço IP do computador com X-Plane
  uint16_t simPort;     // Porta UDP usada pelo simulador (normalmente 49000)

  // Função que constrói e envia um pacote "DREF" com um valor float:
  // ref   -> DataRef do simulador (e.g. "sim/cockpit2/controls/yoke_roll_ratio")
  // value -> valor a ser aplicado (converte float em little-endian)
  // Formato do pacote:
  //   "DREF", 1 byte padding, 4 bytes valor, string com dataref, '\0'
  void sendDREF(const char* ref, float value);
};
