// XPCProtocol.h
#pragma once  // Garante que o arquivo só seja incluído uma vez por compilação

// Bibliotecas necessárias
#include <WiFiUdp.h>       // Suporte à comunicação UDP no ESP32
#include <vector>          

// Classe responsável por gerenciar a comunicação com o plugin XPlaneConnect
// usando o protocolo GETD e RESP.
class XPCProtocol {
public:
  // Inicializa o objeto definindo o IP e a porta do plugin XPlaneConnect
  // ip       -> Endereço IP do computador rodando X-Plane
  // port     -> Porta UDP configurada no plugin XPlaneConnect (ex: 49009)
  void begin(const char* ip, uint16_t port);

  // Envia uma requisição GETD especificando uma lista de DataRefs
  // Exemplo de uso:
  //   xpc.sendGETD({"sim/flightmodel/position/local_ax", ...});
  void sendGETD(const std::vector<const char*>& datarefs);

  // Reenvia a última requisição GETD (sem alterar a lista)
  void sendGETD();

  // Lê as respostas RESP do XPlaneConnect
  void handleResponses();

  float local_ax, local_ay, local_az; // acelerações (em m/s²)
  float acc_hz;                        // frequência (Hz) com que as acelerações estão sendo lidas

private:
  WiFiUDP udp;               // Socket UDP para comunicação com o plugin
  IPAddress xpcIP;           // IP do X-Plane (onde roda o plugin XPlaneConnect)
  uint16_t xpcPort;          // Porta UDP configurada no plugin

  // Lista de DataRefs a serem requisitados
  // Armazena os nomes em String
  std::vector<String> refs;

  // Processamento dos pacotes RESP vindos do XPlaneConnect
  // buf -> ponteiro para os dados do pacote
  // len -> tamanho do pacote
  // Esse método faz o parse dos bytes, decodifica floats e atualiza ax/ay/az
  void parseRESP(uint8_t* buf, int len);
};
