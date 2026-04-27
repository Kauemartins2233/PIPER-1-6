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
├── README.md                                  (este arquivo)
│
├── HIL_simulink/                              ← v1: só controle (validado)
│   ├── arduino_controlador_manual/
│   │   └── arduino_controlador_manual.ino     ← firmware: PIDs Tustin + SAS
│   ├── modeloNL1_HIL.slx                      ← Simulink: planta NL + ponte
│   ├── hil_serial_step.m                      ← bridge USB (com log CSV)
│   ├── ping_arduino.m                         ← teste 1: sanity check (~5 s)
│   ├── run_hil_test.m                         ← teste 2: sim no trim (~50 s)
│   ├── run_hil_manobra.m                      ← teste 3: 3 manobras (~3 min)
│   └── view_hil_log.m                         ← plota hil_log.csv
│
└── guiagem embarcada/                         ← v2: controle + guiagem LOS
    ├── README.md
    ├── arduino_guiagem_controle/
    │   └── arduino_guiagem_controle.ino       ← firmware: PIDs + LOS + WPs
    ├── modelo_HIL_guiagem.slx                 ← Simulink: planta + tap xN,xE
    ├── hil_serial_step_guiagem.m              ← bridge + upload de WPs
    ├── gui_waypoints_HIL.m                    ← GUI pré-missão
    └── tools/
        ├── criar_modelo_HIL_guiagem.m         ← regenera o .slx do zero
        └── fix_sample_time.m                  ← fix do bug RK4 (após regenerar)
```

> **Atenção:** a v2 tem uma divergência conhecida no ganho `K_PSI` do
> heading hold (firmware: 0.3 vs comentário: 0.8). Confirmar antes de
> usar pra missões.

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
arduino-cli upload  --fqbn arduino:avr:mega -p COM4 Arduino/HIL_simulink/arduino_controlador_manual
```

ou, pra v2:

```
arduino-cli compile --fqbn arduino:avr:mega "Arduino/guiagem embarcada/arduino_guiagem_controle"
arduino-cli upload  --fqbn arduino:avr:mega -p COM4 "Arduino/guiagem embarcada/arduino_guiagem_controle"
```

## Hardware

- Arduino Mega 2560 via cabo USB direto no PC, nada de shield.
- Firmware gravado pela Arduino IDE ou `arduino-cli`. Sem libs externas.
- **Porta COM:** o Windows atribui automaticamente. Veja em
  `Gerenciador de Dispositivos → Portas (COM e LPT)` qual é a sua
  ("Arduino Mega 2560 (COMx)"). No PowerShell:

  ```powershell
  Get-PnpDevice -Class Ports -PresentOnly |
      Where-Object FriendlyName -match 'Arduino' |
      Select FriendlyName
  ```

  Se não for **COM4**, edite a linha 19 de `hil_serial_step.m` (e a v2
  equivalente). Os scripts atuais assumem COM4.
- **Não abra o Serial Monitor** com a sim rodando — o protocolo é
  binário e o Monitor segura a porta.
- Entre runs, se a porta ficar travada, rode `clear hil_serial_step` no
  Command Window do MATLAB e/ou desconecte/reconecte o cabo USB.

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

## Avisos de `SFUNC WARN: NaN/Inf detectado` (benignos)

Durante a simulação `modeloNL1_HIL`, o `sfunction_piper` imprime
`SFUNC WARN t=...: NaN/Inf detectado! u=[NaN NaN NaN NaN]` a cada passo.
**Isso não afeta a dinâmica.** O motivo:

- A S-function tem `DirFeedthrough = 0` (saídas 1-15 só dependem do
  estado).
- Mas em `case 3` (output) ela ainda computa as derivadas das saídas
  16-21 (monitoramento) usando `u`. Como `DirFeedthrough=0`, o Simulink
  não propaga `u` para `mdlOutputs`; ele aparece como NaN.
- A proteção da própria função (`sys(~isfinite(sys)) = 0`) zera o
  monitoramento, e a integração real (`case 1`) recebe `u` correto.

Para silenciar, basta remover o cálculo de `Xp = dyn_rigidbody(...)` em
`case 3` de `modelos/Não Linear/sfunction_piper.m` (linhas 99-103) — as
saídas 16-21 ficam zeradas, sem impacto no controle.

## Testes prontos (`HIL_simulink/`)

Três scripts auxiliares ajudam a validar a bancada antes de mexer no
firmware:

### 1. `ping_arduino.m` — sanity check do link serial

Manda 5 frames de sensores no estado de equilíbrio e imprime o que o
Mega devolve. Útil só pra confirmar que a porta abre, o firmware
responde e a latência está OK.

```matlab
>> cd Arduino/HIL_simulink
>> ping_arduino
```

Esperado: `thr ≈ 0.51`, `elev ≈ -0.02`, `ail = 0`, `rud = 0`, latência
~20-30 ms por passo (após o handshake DTR de ~3 s no 1º).

### 2. `run_hil_test.m` — simulação do trim

Roda `modeloNL1_HIL` por 30 s partindo do equilíbrio e reporta std
dos últimos 10 s. Esperado: `Alt std < 0.001 m`, `VT std < 0.001 m/s`
(condição "fácil", aeronave já no trim).

```matlab
>> cd Arduino/HIL_simulink
>> run_hil_test
```

### 3. `run_hil_manobra.m` — três cenários transientes

Perturba a condição inicial e mede a resposta do controlador. Gera
`manobras_hil_v1.png` com Alt(t) e VT(t) para cada cenário:

| # | Perturbação | Alvo |
|---|---|---|
| 1 | Alt₀ = 110 m | Alt → 100 m |
| 2 | VT₀ = 17 m/s | VT → 15.11 m/s |
| 3 | φ₀ = 20° | φ → 0 |

```matlab
>> cd Arduino/HIL_simulink
>> run_hil_manobra
```

Esperado para o cenário 1 (replica o README clássico):

```
Alt: 110.00 → min 98.59 → final 100.0  std(últ 10 s) ≈ 0.01 m
```

### 4. `view_hil_log.m` — inspeção do tráfego serial

`hil_serial_step.m` grava `hil_log.csv` com cada frame trocado com o
Mega (timestamp, 15 sensores enviados, 4 atuadores recebidos). Útil pra
verificar exatamente o que o firmware está mandando de volta.

```matlab
>> run_hil_test                  % gera/atualiza hil_log.csv (~3000 linhas/30s)
>> clear hil_serial_step         % flush + close do CSV
>> view_hil_log                  % plota sensores e atuadores em subplots
```

Colunas do CSV: `t,p,q,r,u,v,w,phi,theta,psi,VT,alpha,beta,sens13,sens14,alt,thr,elev,ail,rud`.
Truncado a cada nova sim. `clear hil_serial_step` é necessário antes
de plotar (fecha o file handle e flusha buffers).

## Convergência v1 (Xe(12) = -110, Ts = 0.01, 30 s)

```
Alt: 110.00 → min 98.59 → final 100.00   std(últ 10 s) = 0.01 m
VT : 15.11  → min 14.07 → final 15.11    std(últ 10 s) = 0.02 m/s
```
