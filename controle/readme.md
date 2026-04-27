# Controle - Piper J-3 Cub 1/6

Malhas de controle longitudinal e látero-direcional projetadas para a aeronave Piper J-3 Cub 1/6, usando controladores PID contínuos projetados via PID Tuner do MATLAB.

## Estrutura

```
controle/
  Linear/          -- Modelo Simulink com malhas de controle aplicadas ao modelo linear
  Não Linear/      -- Modelo Simulink com malhas de controle aplicadas ao modelo não linear
```

## Arquitetura do Controle

O projeto segue a abordagem de (STEVENS & LEWIS, 2016) e (SANTOS, 2018), com controladores PID contínuos projetados via PID Tuner do MATLAB. Para implementação em microcontrolador (ESP32/Arduino) são discretizados pelo método de Tustin (Ts = 0.01 s, 100 Hz).

### Longitudinal

Três malhas em cascata/paralelo:

```
                          Altitude Hold
h_ref ──> Sum(+-) ──> PID(s) ──> Add(+ Xe(8)) ──> Theta_ref
[alt] ─────┘                                          |
                                                       v
                   Longitudinal Attitude Control
Theta_ref ──> Sum(+-) ──> PID(s) ──> Sum(+-) ──> Sat_Elevator ──> delta_e
[theta] ──────┘                      Kq*[q] ──┘

                    Velocity Control
VT_ref ──> Sum(+-) ──> PID(s) ──> Sat_Throttle ──> delta_T
[VT] ──────┘
```

| Malha | Função | Tipo | P | I | D | N | Saturação | Anti-Windup |
|-------|--------|------|---|---|---|---|-----------|-------------|
| Altitude Hold | h_ref → theta_ref | PID | 0.08 | 0.02 | 0.0 | 20 | [-0.17, 0.26] | clamping |
| Pitch (Atitude) | theta_ref → delta_e | PID | 0.260 | 0.143 | 0.0 | 20 | [-0.4363, 0.4363] | clamping |
| SAS Arfagem | Amortecimento de q | Ganho | Kq = 0.1 | - | - | - | - | - |
| Velocidade | VT_ref → delta_T | PID | 0.05 | 0.02 | 0.01 | 20 | [-0.49, 0.51] | clamping |

### Saturações dos Atuadores

Blocos de saturação inseridos entre os controladores e a planta, correspondentes aos limites físicos do Piper 1/6:

| Atuador | Limites | Unidade |
|---------|---------|---------|
| Sat_Throttle | [0, 1] | adimensional |
| Sat_Elevator | [-0.4363, 0.4363] | rad (±25°) |
| Sat_Aileron | [-0.4363, 0.4363] | rad (±25°) |
| Sat_Rudder | [-0.4363, 0.4363] | rad (±25°) |

### Látero-direcional

Duas malhas:

```
                PA de Rolamento (Bank Angle Hold)
psi_ref ──> Sum(+-) ──> Gain(0.3) ──> Sum(+-) ──> PID(s) ──> Sum(+-) ──> Sat_Aileron ──> delta_a
[psi] ─────┘                          [phi] ──────┘           Kp*[p] ──┘

                PA de Guinada (Heading Select)
[r] ──> Washout [s/(s+1)] ──> Kr ──> Sum(+-) ──> Sat_Rudder ──> delta_r
                                      0 ────────┘
```

| Malha | Função | Tipo | P | I | D | N | Saturação | Anti-Windup |
|-------|--------|------|---|---|---|---|-----------|-------------|
| Roll (Bank Angle) | phi_ref → delta_a | PID | 10.0 | 0.0 | 0.0 | 20 | [-0.43, 0.43] | back-calculation |
| SAS Rolamento | Amortecimento de p | Ganho | Kp = 0.119 | - | - | - | - | - |
| Heading → phi | psi_ref → phi_ref | Ganho | 0.3 | - | - | - | - | - |
| Amortecedor Guinada | Washout de r | Ganho + Filtro | Kr = 0.15 | - | - | - | filtro s/(s+1) | - |

### Discretização (para microcontrolador)

Método de Tustin, Ts = 0.01 s (100 Hz):

```
elevator: u[k] = u[k-1] + b0*e[k] + b1*e[k-1] - Kq*q
aileron:  u[k] = u[k-1] + b0*e[k] + b1*e[k-1] - Kp*p
rudder:   u[k] = u[k-1] - Kr*r
```

## Modos Dinâmicos

### Longitudinal

| Modo | Autovalores | wn (rad/s) | Amortecimento | Período (s) |
|------|-------------|------------|---------------|-------------|
| Período Curto | -3.81 ± 5.22j | 6.45 | 0.59 | 1.0 |
| Período Longo (Fugóide) | -0.04 ± 0.18j | 0.184 | 0.22 | 34.1 |

### Látero-direcional

| Modo | Autovalores | wn (rad/s) | Amortecimento | Período (s) |
|------|-------------|------------|---------------|-------------|
| Dutch Roll | -1.19 ± 4.03j | 4.20 | 0.28 | 1.56 |
| Espiral | 0.039 | 0.039 | -1 | - |
| Rolamento Puro | -27.85 | 27.85 | 1 | - |

## Validação

As malhas foram validadas em três cenários:
1. **Modelo linear** -- resposta de referência
2. **Modelo não linear** -- verificação de representatividade (resultados consistentes)
3. **X-Plane** -- plataforma HIL (apresentou divergências; trabalho futuro)

Os modelos linear e não linear apresentaram respostas praticamente sobrepostas para manobras suaves.
