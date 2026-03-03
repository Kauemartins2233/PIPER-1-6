# Arduino / ESP32 - Interface com X-Plane

Firmware para comunicacao entre microcontroladores (Arduino/ESP32) e o simulador X-Plane via protocolo UDP.

## Subdiretorios

| Pasta | Descricao |
|-------|-----------|
| `arduino_connect/` | Versao para Arduino com Ethernet Shield (comunicacao UDP via SPI) |
| `esp32_connect/` | Versao para ESP32 com Wi-Fi (mesmo controlador, sem Ethernet Shield) |
| `esp32_udp_datarefs/esp32connect/` | Versao completa com leitura de DataRefs via plugin XPlaneConnect |

## Controladores implementados

Os firmwares implementam controladores PI discretos (Tustin, Ts = 0.01 s) com as mesmas malhas projetadas no MATLAB:
- **Longitudinal:** altitude, velocidade, pitch + SAS de arfagem
- **Latero-direcional:** roll + SAS de rolagem e washout de guinada
