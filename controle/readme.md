# Controle - Piper J-3 Cub 1/6

Malhas de controle longitudinal e latero-direcional projetadas para a aeronave Piper J-3 Cub 1/6, usando Teoria Classica de Controle (controladores PI) sobre os modelos linearizados.

## Estrutura

```
controle/
  Linear/          -- Modelo Simulink com malhas de controle aplicadas ao modelo linear
  Nao Linear/      -- Modelo Simulink com malhas de controle aplicadas ao modelo nao linear
```

## Arquitetura do Controle

O projeto segue a abordagem de (STEVENS & LEWIS, 2016) e (SANTOS, 2018), com controladores PI continuos projetados via PID Tuner do MATLAB e discretizados pelo metodo de Tustin (Ts = 0.01 s, 100 Hz) para implementacao em microcontrolador (ESP32/Arduino).

### Longitudinal

Tres malhas em cascata/paralelo (Figura 18 do Trabalho 3):

| Loop | Funcao | Entrada | Saida | Ganhos (Kp / Ki) |
|------|--------|---------|-------|-------------------|
| Loop 1 (SAS Arfagem) | Aumento de estabilidade de pitch | q | delta_e | Realimentacao de q |
| Loop 2 (Velocidade) | Rastreamento de VT | VT_ref - VT | delta_T | 0.00644 / 0.0548 |
| Loop 3 (Altitude) | Controle de altitude | h_ref - h | theta_ref | 0.222 / 0.00146 |
| Pitch | Rastreamento de theta | theta_ref - theta | delta_e | 0.632 / 0.967 |

Lei de controle discretizada (forma incremental):

```
u[k] = u[k-1] + b0*e[k] + b1*e[k-1] - Kq*q
```

### Latero-direcional

Duas malhas (Figura 20 do Trabalho 3):

| Malha | Funcao | Entrada | Saida | Ganhos (Kp / Ki) |
|-------|--------|---------|-------|-------------------|
| SAS Rolamento | Amortecedor de rolagem (realimentacao de p) | p | delta_a | K = 0.1 |
| Roll (PI) | Rastreamento de phi | phi_ref - phi | delta_a | 30.5907 / 68.7893 |
| Rudder | Amortecedor de guinada | r | delta_r | Kr (washout) |

Leis de controle discretizadas:

```
aileron:  u[k] = u[k-1] + b0*e[k] + b1*e[k-1] - Kp*p
rudder:   u[k] = u[k-1] - Kr*r
```

## Modos Dinamicos

### Longitudinal

| Modo | Autovalores | wn (rad/s) | Amortecimento | Periodo (s) |
|------|-------------|------------|---------------|-------------|
| Periodo Curto | -3.81 +/- 5.22j | 6.45 | 0.59 | 1.0 |
| Periodo Longo (Fugoide) | -0.04 +/- 0.18j | 0.184 | 0.22 | 34.1 |

### Latero-direcional

| Modo | Autovalores | wn (rad/s) | Amortecimento | Periodo (s) |
|------|-------------|------------|---------------|-------------|
| Dutch Roll | -1.19 +/- 4.03j | 4.20 | 0.28 | 1.56 |
| Espiral | 0.039 | 0.039 | -1 | - |
| Rolamento Puro | -27.85 | 27.85 | 1 | - |

## Validacao

As malhas foram validadas em tres cenarios:
1. **Modelo linear** -- resposta de referencia
2. **Modelo nao linear** -- verificacao de representatividade (resultados consistentes)
3. **X-Plane** -- plataforma HIL (apresentou divergencias; trabalho futuro)

Os modelos linear e nao linear apresentaram respostas praticamente sobrepostas para manobras suaves.
