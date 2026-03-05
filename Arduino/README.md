# Arduino / ESP32 - Interface com X-Plane

Firmware para comunicação entre microcontroladores (Arduino/ESP32) e o simulador X-Plane via protocolo UDP.

## Subdiretórios

| Pasta | Descrição |
|-------|-----------|
| `arduino_connect/` | Versão para Arduino com Ethernet Shield (comunicação UDP via SPI) |
| `esp32_connect/` | Versão para ESP32 com Wi-Fi (mesmo controlador, sem Ethernet Shield) |
| `esp32_udp_datarefs/esp32connect/` | Versão completa com leitura de DataRefs via plugin XPlaneConnect |

## Controladores implementados

Os firmwares implementam controladores PI discretos (Tustin, Ts = 0.01 s) com as mesmas malhas projetadas no MATLAB:
- **Longitudinal:** altitude, velocidade, pitch + SAS de arfagem
- **Látero-direcional:** roll + SAS de rolagem e washout de guinada
