# Guiagem - Piper J-3 Cub 1/6

Sistema de guiagem por waypoints integrado ao modelo não linear 6-DOF com piloto automático.

## Arquivos

| Arquivo | Descrição |
|---------|-----------|
| `NL_guidance.slx` | Modelo Simulink: guiagem + controle + planta não linear |
| `plot3d_voo.m` | Plota trajetória 3D após simulação |
| `plots_guidance.mlx` | Live Script de plotagem para validação (3 testes) |

## Como rodar

1. Na raiz do repositório, rodar `inicializar` (carrega parâmetros, ganhos e waypoints).
2. Abrir `guiagem/NL_guidance.slx` no Simulink.
3. (Opcional) Alterar `missao` no `inicializar.m` para selecionar o perfil de voo.
4. Simular (Ctrl+T).
5. Rodar `plot3d_voo` para visualizar a trajetória 3D.

```matlab
>> inicializar
>> open('guiagem/NL_guidance.slx')
>> % Simular (Ctrl+T)
>> plot3d_voo
```

## Missões disponíveis

Selecionar via variável `missao` no `inicializar.m`:

| Missão | Descrição |
|--------|-----------|
| 1 | Voo reto nivelado (teste de controle puro, sem transições de WP) |
| 2 | Triângulo com altitude constante (4 waypoints a 100m) |
| 3 | Triângulo com variação de altitude (100m → 120m → 90m → 100m) |

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
                |  Modelo não linear 6-DOF         |
                |  21 outputs, DirFeedthrough=0    |
                +----------------------------------+
```

## Algoritmo de Guiagem: Guidance_Star

**Tipo:** Line-of-Sight (LOS) puro com navegação por waypoints.

**Entradas:**
- `pos_n, pos_e` -- posição atual NED (m), feedback da planta
- `WPs` -- matriz de waypoints `[Norte, Leste, Altitude, Velocidade]`
- `R_accept` -- raio de aceitação para troca de waypoint (m)
- `Xe_init` -- vetor de estado inicial 12x1

**Saídas:**
- `psi_ref` -- heading desejado via `atan2(delta_e, delta_n)`
- `h_ref` -- altitude do waypoint alvo
- `v_ref` -- velocidade do waypoint alvo
- `wp_idx_monitor` -- índice do waypoint atual
- `dist_monitor` -- distância até o alvo

**Lógica de troca de waypoint:**
Quando `dist_error <= R_accept` e não é o último WP, avança para o próximo.

### Função auxiliar: calc_erro_proa

Normaliza o erro de heading para `[-pi, +pi]` garantindo que a aeronave vire pelo lado mais curto:

```matlab
erro_psi = mod(diff + pi, 2*pi) - pi;
```

## Ganhos do Piloto Automático

Os ganhos são carregados do workspace via `inicializar.m`. Os mesmos valores absolutos são usados no `modeloNL1.slx` (controle). Os blocos PID no modelo referenciam diretamente as variáveis do workspace.

### Longitudinal

| Malha | Variável | Tipo | P | I | D | N | Sat |
|-------|----------|------|---|---|---|---|-----|
| Altitude Hold | `C_alt` | PID | 0.596 | 0.356 | -0.0142 | 6.17 | [-0.17, 0.26] |
| Pitch (Atitude) | `C_theta` | PID | 20.31 | 22.60 | 1.767 | 1159.4 | sem |
| SAS Arfagem | `Kq` | Ganho | 0.1 | - | - | - | - |
| Velocidade | `C_vel` | PID | 0.0787 | 0.0200 | 0.0152 | 77.0 | sem |

### Látero-direcional

| Malha | Variável | Tipo | P | I | D | N | Sat |
|-------|----------|------|---|---|---|---|-----|
| Roll (Bank Angle) | `C_phi` | PID | 26.79 | 13.17 | -0.0876 | 305.9 | [-0.43, 0.43] |
| Heading | - | P | 2.5 | 0 | 0 | - | [-1.0, 1.0] |
| SAS Rolamento | `Kp` | Ganho | 0.119 | - | - | - | - |
| Heading -> phi | - | Ganho | 0.3 | - | - | - | - |
| Amortecedor Guinada | `Kr` | Ganho + Washout | 0.15 | - | - | - | filtro s/(s+1) |

## Dependências

Todas carregadas por `inicializar.m`:
- `par_aero`, `par_prop`, `par_gen` (structs de parâmetros da aeronave, via .mat)
- `Xe` (vetor de estado de equilíbrio 12x1, via `equilibrium.m`)
- `TrimInput` (vetor de comandos de equilíbrio `[thr, elev, ail, rud]`)
- `WPs` (matriz de waypoints `[Norte, Leste, Alt, Vel]`)
- `R_accept` (raio de aceitação em metros)
- `C_alt`, `C_theta`, `C_vel`, `C_phi` (structs com ganhos PID)
- `Kq`, `Kp`, `Kr` (ganhos dos SAS e amortecedores)
- `Xe_init` (estado inicial passado ao Guidance_Star)

**Nota:** O modelo usa `sfunction_piper` (21 outputs, DirFeedthrough=0), o mesmo arquivo usado pelo modeloNL1.slx.

## Testes de Validação (plots_guidance.mlx)

O Live Script gera 3 figuras:

1. **Teste 1 -- Longitudinal:** Rastreamento de altitude + desacoplamento lateral + atuadores
2. **Teste 2 -- Lateral (Dogleg):** Rastreamento de heading + aileron + manutenção de altitude em curva
3. **Teste 3 -- Acoplado:** Altitude + heading + superfícies de controle simultâneos
