# Guiagem - Piper J-3 Cub 1/6

Sistema de guiagem por waypoints integrado ao modelo nao linear 6-DOF com piloto automatico.

## Arquivos

| Arquivo | Descricao |
|---------|-----------|
| `NL_guidance.slx` | Modelo Simulink: guiagem + controle + planta nao linear |
| `plots_guidance.mlx` | Live Script de plotagem para validacao (3 testes) |

## Arquitetura

```
                 ┌────────────────┐
  WPs ──────────┤                ├── psi_ref ──┐
  pos_n, pos_e ─┤  Guidance_Star ├── h_ref ────┤
  R_accept ─────┤  (MATLAB Func) ├── v_ref ────┤
  Xe_init ──────┤                ├── wp_idx ───┤ (monitor)
                └────────────────┘              │
                                                v
                ┌───────────────────────────────────────────┐
                │              Autopilot1                    │
                │  Altitude Hold --> Pitch Control --> Elev  │
                │  Velocity Control ----------------> Thr   │
                │  Heading --> Roll Control ---------> Ail   │
                │  Washout Guinada -----------------> Rud   │
                └────────────────────┬──────────────────────┘
                                     v
                ┌────────────────────────────────┐
                │  Piper_1_6 (sfunction_piper)   │
                │  Modelo nao linear 6-DOF       │
                │  21 outputs                    │
                └────────────────────────────────┘
```

## Algoritmo de Guiagem: Guidance_Star

**Tipo:** Line-of-Sight (LOS) puro com navegacao por waypoints.

**Entradas:**
- `pos_n, pos_e` -- posicao atual NED (m), feedback da planta
- `WPs` -- matriz de waypoints `[Norte, Leste, Altitude, Velocidade]`
- `R_accept` -- raio de aceitacao para troca de waypoint (m)
- `Xe_init` -- vetor de estado inicial 12x1

**Saidas:**
- `psi_ref` -- heading desejado via `atan2(delta_e, delta_n)`
- `h_ref` -- altitude do waypoint alvo
- `v_ref` -- velocidade do waypoint alvo
- `wp_idx_monitor` -- indice do waypoint atual
- `dist_monitor` -- distancia ate o alvo

**Logica de troca de waypoint:**
Quando `dist_error <= R_accept` e nao e o ultimo WP, avanca para o proximo.

### Funcao auxiliar: calc_erro_proa

Normaliza o erro de heading para `[-pi, +pi]` garantindo que a aeronave vire pelo lado mais curto:

```matlab
erro_psi = mod(diff + pi, 2*pi) - pi;
```

## Ganhos do PA (neste modelo)

Os ganhos sao parametrizados e carregados do workspace (`C_alt`, `C_theta`, `C_vel`, `C_phi`, `Kq`, `Kp`, `Kr`).

### Longitudinal

| Malha | Tipo | Ganhos | Sat |
|-------|------|--------|-----|
| Altitude Hold | PI | C_alt.Kp, C_alt.Ki | [-0.17, 0.26] |
| Pitch (Atitude) | PI | C_theta.Kp, C_theta.Ki | sem |
| SAS Arfagem | Ganho | Kq | - |
| Velocidade | PI | C_vel.Kp, C_vel.Ki | sem |

### Latero-direcional

| Malha | Tipo | Ganhos | Sat |
|-------|------|--------|-----|
| Roll (Bank Angle) | PI | C_phi.Kp, C_phi.Ki | [-0.43, 0.43] |
| Heading (PID) | PI | P=2.0, I=0.9 | sem |
| SAS Rolamento | Ganho | Kp | - |
| Heading -> phi | Ganho | 0.15 | - |
| Amortecedor Guinada | Ganho + Washout | Kr, filtro s/(s+1) | - |

## Dependencias

Antes de rodar, e necessario ter no workspace:
- `par_aero`, `par_prop`, `par_gen` (structs de parametros da aeronave)
- `Xe` (vetor de estado de equilibrio, 12x1)
- `TrimInput` (vetor de comandos de equilibrio `[thr, elev, ail, rud]`)
- `WPs` (matriz de waypoints)
- `C_alt`, `C_theta`, `C_vel`, `C_phi` (structs com ganhos dos PIDs)
- `Kq`, `Kp`, `Kr` (ganhos dos SAS e amortecedores)

**Nota:** O modelo usa `sfunction_piper` com 21 outputs (mesma versao do trim). Certifique-se de que a pasta `trim/` esta no path ou use `sfunction_piper_trim`.

## Testes de Validacao (plots_guidance.mlx)

O Live Script gera 3 figuras:

1. **Teste 1 -- Longitudinal:** Rastreamento de altitude + desacoplamento lateral + atuadores (elevator vs aileron)
2. **Teste 2 -- Lateral (Dogleg):** Rastreamento de heading + aileron + manutencao de altitude em curva
3. **Teste 3 -- Acoplado:** Altitude + heading + superficies de controle simultaneos
