# HIL Arduino Mega — PIPER 1/6

Bancada Hardware-in-the-Loop com Arduino Mega 2560 embarcando o
controlador (PIDs + SAS) e, opcionalmente, a guiagem por waypoints (LOS).
Planta não-linear do Piper 1/6 roda no Simulink do PC. Comunicação via
**USB Serial @ 115200 baud**, 100 Hz.

Inspirado no HIL da dissertação do Marcelo H. Santos (2018), com
diferenças: Arduino Mega (ATmega2560 8-bit) no lugar do Stellaris
Cortex-M3, e USB Serial no lugar de Ethernet/UDP.

## Duas versões

| | `HIL_simulink/` | `guiagem embarcada/` |
|---|---|---|
| Escopo | só controle embarcado | controle **+ guiagem LOS** embarcados |
| Refs `h_ref`, `V_ref` | hardcoded no firmware | vêm dos waypoints |
| Heading hold (ψ→φ_ref) | não tem (φ_ref=0) | sim (filtro + sat + K=0.3) |
| Waypoints | n/a | upload 1 vez no início via serial |
| Sensores[13,14] | zerados | xN, xE |
| GUI pré-missão | não tem | `gui_waypoints_HIL` |
| Uso | validação/comparação de controle | missões completas |

As duas pastas são **independentes** e podem coexistir. A v1 continua como
referência para validação do controle puro.

## Estrutura

```
Arduino/
├── README.md                              (este arquivo)
│
├── HIL_simulink/                          ← v1: só controle
│   ├── README.md
│   ├── arduino_controlador_manual/
│   │   └── arduino_controlador_manual.ino    (firmware PIDs Tustin + SAS)
│   ├── modeloNL1_HIL.slx                     (planta NL + ponte serial)
│   ├── hil_serial_step.m                     (ponte MATLAB↔Mega)
│   └── tools/
│       ├── criar_modeloNL_HIL.m              (gera o .slx do zero)
│       ├── fix_sample_time.m                 (fix do bug RK4)
│       ├── count_calls.m                     (debug de chamadas/passo)
│       ├── adicionar_scopes.m                (adiciona Scopes ao .slx)
│       ├── run_hil_test.m                    (sim batch)
│       └── test_transport_delay.m            (experimento delay)
│
└── guiagem embarcada/                     ← v2: controle + guiagem
    ├── README.md
    ├── arduino_guiagem_controle/
    │   └── arduino_guiagem_controle.ino      (firmware + LOS + WP storage)
    ├── modelo_HIL_guiagem.slx                (planta NL + ponte + tap xN,xE)
    ├── hil_serial_step_guiagem.m             (ponte + upload WPs)
    ├── gui_waypoints_HIL.m                   (GUI pré-missão)
    └── tools/
        ├── criar_modelo_HIL_guiagem.m        (gera o .slx do zero)
        └── fix_sample_time.m                 (fix do bug RK4)
```

## Uso rápido

### v1 — só controle (`HIL_simulink/`)

```matlab
inicializar
open_system('modeloNL1_HIL')        % clique Run
```

### v2 — controle + guiagem (`guiagem embarcada/`)

```matlab
inicializar
addpath('Arduino/guiagem embarcada')
gui_waypoints_HIL                   % clique no mapa + SIMULAR (HIL)
```

Grave o firmware correspondente antes:

```
arduino-cli compile --fqbn arduino:avr:mega Arduino/HIL_simulink/arduino_controlador_manual
arduino-cli upload  --fqbn arduino:avr:mega -p COM3 Arduino/HIL_simulink/arduino_controlador_manual
```

ou, pra v2:

```
arduino-cli compile --fqbn arduino:avr:mega "Arduino/guiagem embarcada/arduino_guiagem_controle"
arduino-cli upload  --fqbn arduino:avr:mega -p COM3 "Arduino/guiagem embarcada/arduino_guiagem_controle"
```

## Hardware

- Arduino Mega 2560 em **COM3**.
- Cabo USB direto no PC, nada de shield.
- Firmware gravado pela Arduino IDE ou `arduino-cli`. Sem libs externas.
- **Não abra o Serial Monitor** com a sim rodando — o protocolo é
  binário e o Monitor segura a porta.
- `inicializar.m` libera COM3 + `clear hil_serial_step*` no início.

## Protocolo serial

Binário little-endian @ 115200.

### v1 (só controle)

```
PC  → MEGA :  [0xAA 0x55] + 15 floats   (62 B)
              sensors = [p q r u v w phi theta psi VT alpha beta 0 0 alt]
MEGA → PC  :  [0x55 0xAA] + 4 floats    (18 B)
              U = [thr elev ail rud]
```

### v2 (controle + guiagem)

Igual à v1, mas `sensors[13]=xN`, `sensors[14]=xE` (antes zerados), mais
um frame adicional de upload de WPs no início:

```
PC  → MEGA  WP_UPLOAD  : [0xBB 0x66] + uint8 N + N×(4 floats)   (3+16N B)
                         WP = [N, E, alt, V]   (max N=32)
MEGA → PC   WP_ACK     : [0x66 0xBB] + uint8 N                  (3 B)
```

Sem CRC (link USB já tem checksum em hardware). Uma transação por passo
de 10 ms após o setup.

## Controlador embarcado

Ambas as versões usam os mesmos 5 PIDs+SAS, ganhos do `inicializar.m`:

| Malha | Estrutura | Kp | Ki | Kd | N |
|---|---|---|---|---|---|
| Altitude → θ_ref | PID | 0.08 | 0.02 | 0.0 | 20 |
| θ → elev | PID + `-Kq·q` | 0.260 | 0.143 | 0.0 | 20 |
| Velocidade → thr | PID | 0.05 | 0.02 | 0.01 | 20 |
| φ → ail | PID + `-Kp_sas·p` | 10.0 | 0.0 | 0.0 | 20 |
| Yaw-damper → rud | `-Kr · washout(r)`, τ=1 s | — | — | — | — |

Discretização **Tustin**, anti-windup por **back-calculation**.
Saturações: elev/ail/rud ±0.4363 rad (±25°), throttle [0,1].

A v2 adiciona heading hold (idêntico ao `NL_guidance.slx`):

```
psi_ref → 1/(s+1) → wrap_pi(psi_ref_filt - psi) → sat[-1,+1] → ×0.3 → phi_ref
```

e guiagem LOS (idêntica ao `guiagem/guidance_star.m`):

```c
psi_ref = atan2(WPs[idx].E - xE, WPs[idx].N - xN);
if (dist <= R_accept && idx < N-1) idx++;
```

`R_accept = 80 m` hardcoded, max 32 WPs.

## Detecção de perda de conexão

Ambas as pontes (`hil_serial_step.m` e `hil_serial_step_guiagem.m`)
toleram até 5 passos consecutivos (50 ms) sem resposta usando `Ue`
como fallback, e abortam a sim com erro depois disso.

## Bug crítico resolvido — SampleTime do MATLAB Function

O MATLAB Function block `hil_io` precisa ter `SampleTime = 0.01`
explicitamente, senão herda "contínuo" e é reavaliado nos **4
sub-passos do RK4 (ODE4)** — os PIDs discretos do firmware avançam
então a 400 Hz em vez de 100 Hz, causando ciclo limite grande
(Alt std ≈ 2.5 m). Os `.slx` salvos no repo já estão corrigidos. Se
regenerar com `criar_*`, rode em seguida `fix_sample_time` da pasta
`tools/` correspondente.

Com o fix: Alt std nos últimos 10 s ≈ 0.01 m.

## Convergência v1 (Xe(12) = -110, Ts = 0.01, 30 s)

```
Alt: 110.00 → min 98.59 → final 100.00   std(últ 10 s) = 0.01 m
VT : 15.11  → min 14.07 → final 15.11    std(últ 10 s) = 0.02 m/s
```
