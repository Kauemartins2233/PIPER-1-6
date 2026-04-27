# HIL Arduino Mega — Controle + Guiagem embarcados

Estende `Arduino/HIL_simulink/` com **guiagem por waypoints (LOS)** rodando no
próprio Arduino Mega. PC fica só com a planta NL e a GUI pré-missão.

## Estrutura

```
guiagem embarcada/
├── README.md
├── arduino_guiagem_controle/
│   └── arduino_guiagem_controle.ino     (firmware: PIDs+SAS + LOS + WP storage)
├── modelo_HIL_guiagem.slx               (planta NL + ponte serial; criado por script)
├── hil_serial_step_guiagem.m            (ponte: upload WPs + loop sensores/atuadores)
├── gui_waypoints_HIL.m                  (GUI pré-missão; fork de gui_waypoints.m)
└── tools/
    ├── criar_modelo_HIL_guiagem.m       (gera o .slx do zero)
    ├── fix_sample_time.m                (fix do bug RK4)
    └── run_hil_guiagem_test.m           (batch + plot3d)
```

## Setup inicial (uma vez)

```matlab
inicializar
addpath('Arduino/guiagem embarcada')
addpath('Arduino/guiagem embarcada/tools')
criar_modelo_HIL_guiagem      % gera modelo_HIL_guiagem.slx
fix_sample_time               % aplica fix do RK4 (CRÍTICO)
```

E grave o firmware:

```
arduino-cli compile --fqbn arduino:avr:mega Arduino/guiagem\ embarcada/arduino_guiagem_controle
arduino-cli upload  --fqbn arduino:avr:mega -p COM3 Arduino/guiagem\ embarcada/arduino_guiagem_controle
```

## Uso diário

```matlab
gui_waypoints_HIL
```

Clique no mapa pra colocar waypoints, ajuste altitude/velocidade/StopTime, e
clique **SIMULAR (HIL)**. A GUI:

1. Faz upload da matriz WPs ao Mega via mensagem `WP_UPLOAD` (header `0xBB 0x66`).
2. Aguarda ACK (`0x66 0xBB N`).
3. Roda a planta NL no Simulink em pacing de tempo real, trocando sensores/atuadores
   com o Mega a 100 Hz.
4. Pós-sim chama `plot3d_voo` (3 figuras: trajetória 3D, ground track, alt×t).

Limite: **32 waypoints** (após interpolação automática em altitude).

## Protocolo serial (115200 baud, little-endian)

```
PC → MEGA  WP_UPLOAD  : [0xBB 0x66] uint8 N + N×(4 floats)   (3 + 16N B)
                        WP = [N, E, alt, V]
MEGA → PC  WP_ACK     : [0x66 0xBB] uint8 N                  (3 B)

PC → MEGA  SENSORS    : [0xAA 0x55] + 15 floats              (62 B)
  sensors = [p q r u v w phi theta psi VT alpha beta xN xE alt]
                                                    ^^^^^ (no HIL_simulink eram 0 0)
MEGA → PC  ACTUATORS  : [0x55 0xAA] + 4 floats               (18 B)
  U = [thr elev ail rud]
```

## Controlador embarcado

Idêntico ao `Arduino/HIL_simulink/`, com **uma malha extra** de heading hold:

| Malha | Estrutura | Ganhos |
|---|---|---|
| Altitude → θ_ref | PID Tustin | 0.08 / 0.02 / 0 |
| θ → elev | PID + `-Kq·q` | 0.260 / 0.143 / 0 |
| Velocidade → thr | PID Tustin | 0.05 / 0.02 / 0.01 |
| **ψ → φ_ref** | 1/(s+1) → wrap → sat[-1,1] → ×K_ψ | **K_ψ = 0.3** (igual NL_guidance.slx) |
| φ → ail | PID + `-Kp_sas·p` | 10.0 / 0 / 0 |
| Yaw-damper → rud | `-Kr·washout(r)` τ=1 s | 0.15 |

Guiagem LOS (igual `guiagem/guidance_star.m`):

```c
ψ_ref = atan2(WPs[idx].E - xE, WPs[idx].N - xN);
if (dist ≤ R_accept && idx < N-1) idx++;
```

`R_accept = 80 m`, hardcoded no firmware. Para mudar, recompile.

## Diferenças em relação a `Arduino/HIL_simulink/`

| | HIL_simulink | guiagem embarcada |
|---|---|---|
| Refs (h_ref, V_ref) | hardcoded no firmware | vêm dos WPs |
| Heading hold | não tem (φ_ref = 0) | sim, K_ψ = 0.3 (c/ filtro + sat) |
| WPs | n/a | upload uma vez no início |
| Sensors[12,13] | 0, 0 | xN, xE |
| GUI | não tem | `gui_waypoints_HIL` |

## Verificação

1. **Sem GUI** (Mission 3 do `inicializar.m`):
   ```matlab
   inicializar
   cd 'Arduino/guiagem embarcada/tools'
   run_hil_guiagem_test
   ```
   Trajetória 3D do `plot3d_voo` deve coincidir com a do `NL_guidance.slx` puro
   dentro de poucos metros.

2. **Bug do RK4**: rodar `count_calls` (do `HIL_simulink/tools`, ajustando o
   nome do modelo) deve dar 1 chamada por passo. Se der 4, rode `fix_sample_time`.

3. **Robustez serial**: desconecte USB no meio da sim → aborto limpo após 50 ms.

## Coexistência

Esta pasta é **independente** de `Arduino/HIL_simulink/`. A versão só-controle
continua existindo lá para validação/comparação.
