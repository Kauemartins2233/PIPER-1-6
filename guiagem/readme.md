# Guiagem - Piper J-3 Cub 1/6

Sistema de guiagem por waypoints integrado ao modelo nao linear 6-DOF com piloto automatico.

## Arquivos

| Arquivo | Descricao |
|---------|-----------|
| `NL_guidance.slx` | Modelo Simulink: guiagem + controle + planta nao linear |
| `setup_pid_blocks.m` | Aplica PID completo (termos D e N) nos blocos do Simulink |
| `plot3d_voo.m` | Plota trajetoria 3D apos simulacao |
| `plots_guidance.mlx` | Live Script de plotagem para validacao (3 testes) |

## Como rodar

1. Na raiz do repositorio, rodar `inicializar` (carrega parametros, ganhos e waypoints).
2. Abrir `guiagem/NL_guidance.slx` no Simulink.
3. Rodar `setup_pid_blocks` (aplica os termos D e N nos blocos PID).
4. (Opcional) Alterar `missao` no `inicializar.m` para selecionar o perfil de voo.
5. Simular (Ctrl+T).
6. Rodar `plot3d_voo` para visualizar a trajetoria 3D.

```matlab
>> inicializar
>> open('guiagem/NL_guidance.slx')
>> setup_pid_blocks
>> % Simular (Ctrl+T)
>> plot3d_voo
```

## Missoes disponiveis

Selecionar via variavel `missao` no `inicializar.m`:

| Missao | Descricao |
|--------|-----------|
| 1 | Voo reto nivelado (teste de controle puro, sem transicoes de WP) |
| 2 | Triangulo (teste de guiagem completo com 4 waypoints) |

## Arquitetura

```
                 +----------------+
  WPs ----------|                |-- psi_ref --+
  pos_n, pos_e -|  Guidance_Star |-- h_ref ----+
  R_accept -----|  (MATLAB Func) |-- v_ref ----+
  Xe_init ------|                |-- wp_idx ---+ (monitor)
                 +----------------+             |
                                                v
                +-------------------------------------------+
                |              Autopilot                     |
                |  Altitude Hold --> Pitch Control --> Elev  |
                |  Velocity Control ----------------> Thr   |
                |  Heading --> Roll Control ---------> Ail   |
                |  Washout Guinada -----------------> Rud   |
                +-----------------+-------------------------+
                                  v
                +----------------------------------+
                |  Piper_1_6 (sfunction_piper)     |
                |  Modelo nao linear 6-DOF         |
                |  21 outputs, DirFeedthrough=0    |
                +----------------------------------+
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

## Ganhos do Piloto Automatico

Os ganhos sao carregados do workspace via `inicializar.m`. Os mesmos valores absolutos sao usados no `modeloNL1.slx` (controle).

**Nota:** Os blocos PID no modelo usam apenas P e I do workspace (D=0, N=100 hardcoded). Para aplicar o PID completo com D e N, rodar `setup_pid_blocks` apos abrir o modelo.

### Longitudinal

| Malha | Variavel | Tipo | P | I | D | N | Sat |
|-------|----------|------|---|---|---|---|-----|
| Altitude Hold | `C_alt` | PID | 0.596 | 0.356 | -0.0142 | 6.17 | [-0.17, 0.26] |
| Pitch (Atitude) | `C_theta` | PID | 20.31 | 22.60 | 1.767 | 1159.4 | sem |
| SAS Arfagem | `Kq` | Ganho | 0.1 | - | - | - | - |
| Velocidade | `C_vel` | PID | 0.0787 | 0.0200 | 0.0152 | 77.0 | sem |

### Latero-direcional

| Malha | Variavel | Tipo | P | I | D | N | Sat |
|-------|----------|------|---|---|---|---|-----|
| Roll (Bank Angle) | `C_phi` | PID | 26.79 | 13.17 | -0.0876 | 305.9 | [-0.43, 0.43] |
| Heading | - | P | 2.5 | 0 | 0 | - | [-1.0, 1.0] |
| SAS Rolamento | `Kp` | Ganho | 0.119 | - | - | - | - |
| Heading -> phi | - | Ganho | 0.3 | - | - | - | - |
| Amortecedor Guinada | `Kr` | Ganho + Washout | 0.15 | - | - | - | filtro s/(s+1) |

**Obs:** Heading (P=2.5, I=0) e Gain1 (0.3) sao configurados via `setup_pid_blocks`.

## Dependencias

Todas carregadas por `inicializar.m`:
- `par_aero`, `par_prop`, `par_gen` (structs de parametros da aeronave, via .mat)
- `Xe` (vetor de estado de equilibrio 12x1, via `equilibrium.m`)
- `TrimInput` (vetor de comandos de equilibrio `[thr, elev, ail, rud]`)
- `WPs` (matriz de waypoints `[Norte, Leste, Alt, Vel]`)
- `R_accept` (raio de aceitacao em metros)
- `C_alt`, `C_theta`, `C_vel`, `C_phi` (structs com ganhos PID)
- `Kq`, `Kp`, `Kr` (ganhos dos SAS e amortecedores)
- `Xe_init` (estado inicial passado ao Guidance_Star)

**Nota:** O modelo usa `sfunction_piper` (21 outputs, DirFeedthrough=0), o mesmo arquivo usado pelo modeloNL1.slx.

## Testes de Validacao (plots_guidance.mlx)

O Live Script gera 3 figuras:

1. **Teste 1 -- Longitudinal:** Rastreamento de altitude + desacoplamento lateral + atuadores
2. **Teste 2 -- Lateral (Dogleg):** Rastreamento de heading + aileron + manutencao de altitude em curva
3. **Teste 3 -- Acoplado:** Altitude + heading + superficies de controle simultaneos
