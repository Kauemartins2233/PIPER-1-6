# ESP32 ↔ X-Plane Interface

Este repositório contém um projeto completo para interligar um **ESP32** ao simulador de voo **X-Plane**, utilizando comunicação UDP tanto com o protocolo padrão (`DATA`) quanto com o plugin **XPlaneConnect** para leitura de DataRefs.

---

## Estrutura do Projeto

- **esp32connect.ino**  
  Arquivo principal. Inicializa a conexão Wi-Fi, configura os módulos e processa o loop principal:
  - Recebe dados de telemetria (`DATA`)
  - Solicita DataRefs via XPlaneConnect (`GETD` / `RESP`)
  - Envia comandos de controle (`DREF`)
    
- **XPlaneData.cpp / XPlaneData.h**  
  Classe responsável por:
  - Receber pacotes de telemetria `DATA` enviados pelo X-Plane

- **XPCProtocol.cpp / XPCProtocol.h**  
  Classe que faz interface com o plugin **XPlaneConnect**:
  - Envia comandos `GETD` para solicitar DataRefs específicos
  - Recebe `RESP` com os valores dos DataRefs solicitados

- **XPlaneControl.cpp / XPlaneControl.h**  
  Classe para enviar comandos de controle ao X-Plane usando o protocolo `DREF`:
  - Comandos incluídos: elevator, aileron, rudder e throttle
  - Entrada contínua de valores normalizados (-1 a 1)

---

## Protocolo Utilizado

Este projeto usa dois protocolos diferentes:

1. **DATA** (padrão X-Plane)  
   Enviado diretamente pelo simulador, habilitado em *Settings → Data Output* (ex: attitude, g-load, GPS etc.).

2. **GETD / RESP** (plugin XPlaneConnect)  
   No caso desse projeto, os DataRefs usados são:  
   - `sim/flightmodel/position/local_ax`  
   - `sim/flightmodel/position/local_ay`  
   - `sim/flightmodel/position/local_az`

---

## Como Usar

1. **Pré-requisitos:**
   - X-Plane instalado (versão 9 ou superior)
   - Plugin [XPlaneConnect](https://github.com/nasa/XPlaneConnect) instalado
   - ESP32 conectado à mesma rede do computador com o simulador

2. **Configurações no X-Plane:**
   - Habilite Data Output via UDP (`DATA`) na porta 49010 (ou altere no código)
   - Certifique-se que o plugin XPlaneConnect está configurado na porta 49009
   - Verifique no console do X-Plane se o plugin está carregado

3. **Compilação e Upload:**
   - Use a IDE Arduino ou PlatformIO
   - Instale as bibliotecas necessárias: `WiFi.h` e `WiFiUdp.h`
   - Faça o upload do sketch para o ESP32

4. **Configuração no Código:**
   No `esp32connect.ino` você pode alterar este bloco conforme sua rede e IP do simulador:

   ```cpp
   const char* ssid = "NOME_DA_SUA_REDE";
   const char* password = "SENHA_DA_REDE";
   const char* SIM_IP = "192.168.x.x"; 
   const uint16_t DATA_PORT = 49010;   
   const uint16_t DREF_PORT = 49000;     
   const uint16_t XPC_PORT  = 49009; 

