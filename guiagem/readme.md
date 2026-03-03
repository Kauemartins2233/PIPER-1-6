# Guiagem - Piper J-3 Cub 1/6

Sistema de guiagem por waypoints integrado ao modelo nao linear 6-DOF com piloto automatico.

## Arquivos

| Arquivo | Descricao |
|---------|-----------|
| `executar.m` | Script de inicializacao: carrega parametros, define ganhos e waypoints |
| `NL_guidance.slx` | Modelo Simulink: guiagem + controle + planta nao linear |
| `plots_guidance.mlx` | Live Script de plotagem para validacao (3 testes) |

## Como rodar

1. Rodar `executar.m` (carrega parametros, ganhos e waypoints no workspace).
2. Abrir `NL_guidance.slx` no Simulink.
3. (Opcional) Editar a matriz `WPs` no workspace para alterar a missao.
4. Rodar a simulacao.
5. Para visualizacao, usar `plots_guidance.mlx`.

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

Os ganhos sao parametrizados e carregados do workspace via `executar.m`. Valores extraidos do modelo nao linear (`controle/Nao Linear/modeloNL.slx`).

**Nota:** Os blocos no modelo de guiagem usam PI (D=0, N=100). Os termos D e N do PID completo estao documentados no `executar.m` para referencia caso se queira converter para PID.

### Longitudinal

| Malha | Variavel | Tipo | P | I | D | N | Sat |
|-------|----------|------|---|---|---|---|-----|
| Altitude Hold | `C_alt` | PID | 0.596 | 0.356 | -0.0142 | 6.17 | [-0.17, 0.26] |
| Pitch (Atitude) | `C_theta` | PID | 20.31 | 22.60 | 1.767 | 1159.4 | sem |
| SAS Arfagem | `Kq` | Ganho | 0.1 | - | - | - | - |
| Velocidade | `C_vel` | PID | -0.0787 | -0.0652 | -0.0152 | 77.0 | sem |

### Latero-direcional

| Malha | Variavel | Tipo | P | I | D | N | Sat |
|-------|----------|------|---|---|---|---|-----|
| Roll (Bank Angle) | `C_phi` | PID | 26.79 | 13.17 | -0.0876 | 305.9 | [-0.43, 0.43] |
| Heading | - | PI | 2.0 | 0.9 | - | - | sem |
| SAS Rolamento | `Kp` | Ganho | 0.119 | - | - | - | - |
| Heading -> phi | - | Ganho | 0.8 | - | - | - | - |
| Amortecedor Guinada | `Kr` | Ganho + Washout | 0.15 | - | - | - | filtro s/(s+1) |

**Obs:** Heading PI (P=2.0, I=0.9) e Heading->phi (0.8) sao hardcoded no modelo de guiagem.

## Dependencias

Todas as dependencias sao carregadas por `executar.m`:
- `par_aero`, `par_prop`, `par_gen` (structs de parametros da aeronave, via .mat)
- `Xe` (vetor de estado de equilibrio 12x1, via `equilibrium.m`)
- `TrimInput` (vetor de comandos de equilibrio `[thr, elev, ail, rud]`)
- `WPs` (matriz de waypoints `[Norte, Leste, Alt, Vel]`)
- `R_accept` (raio de aceitacao em metros)
- `C_alt`, `C_theta`, `C_vel`, `C_phi` (structs com ganhos PI)
- `Kq`, `Kp`, `Kr` (ganhos dos SAS e amortecedores)
- `Xe_init` (estado inicial passado ao Guidance_Star)

**Nota:** O modelo usa `sfunction_piper` com 21 outputs (mesma versao do trim). O script `executar.m` adiciona `trim/` ao path para disponibilizar `sfunction_piper_trim`.

## Testes de Validacao (plots_guidance.mlx)

O Live Script gera 3 figuras:

1. **Teste 1 -- Longitudinal:** Rastreamento de altitude + desacoplamento lateral + atuadores (elevator vs aileron)
2. **Teste 2 -- Lateral (Dogleg):** Rastreamento de heading + aileron + manutencao de altitude em curva
3. **Teste 3 -- Acoplado:** Altitude + heading + superficies de controle simultaneos
