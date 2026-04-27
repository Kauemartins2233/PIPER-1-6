# Guiagem - Piper J-3 Cub 1/6

Sistema de guiagem por waypoints integrado ao modelo não linear 6-DOF com piloto automático.

## Arquivos

| Arquivo | Descrição |
|---------|-----------|
| `NL_guidance.slx` | Modelo Simulink: guiagem LOS + controle + planta não linear |
| `gui_waypoints.m` | Interface visual para inserção de waypoints e simulação automática |
| `plot3d_voo.m` | Plota trajetória 3D pós-simulação |
| `plots_guidance.mlx` | Live Script de plotagem para validação (3 testes) |
| `find_trim.m` | Busca alternativa de trim via `fminsearch` (legado/teste) |
| `trim_cost_fn.m` | Função custo usada por `find_trim.m` |
| `analise_sim.m` | Análise pós-sim: estatísticas e checagens dos controladores |

## Como rodar

### Opção 1: Interface visual (recomendado)

```matlab
>> gui_waypoints
```

A GUI chama `inicializar.m` automaticamente para carregar todos os parâmetros do workspace. Basta clicar no mapa para posicionar waypoints, ajustar altitude e velocidade, e clicar **SIMULAR**.

### Opção 2: Via linha de comando

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

## Interface Visual (gui_waypoints)

Interface gráfica para definição interativa de waypoints. A GUI executa `inicializar.m` internamente para carregar todos os parâmetros necessários.

### Funcionalidades

- **Mapa 2D clicável** (Norte x Leste): clique para adicionar waypoints na posição desejada
- **Tabela editável**: ajuste fino de coordenadas, altitude e velocidade de cada WP
- **Altitude e velocidade padrão**: define os valores aplicados ao próximo clique no mapa
- **Raio de aceitação**: configura o R_accept usado pela guiagem (padrão: 80m)
- **Tempo de simulação**: define o StopTime máximo da simulação (padrão: 200s)
- **Botão Simular**: executa automaticamente a inicialização, simulação e plotagem 3D
- **Remover Último / Limpar Tudo**: botões para gerenciar waypoints

### Interpolação automática de altitude

Quando a diferença de altitude entre dois waypoints consecutivos é maior que 30m, a GUI insere automaticamente waypoints intermediários ao longo do trecho. Isso evita degraus bruscos de altitude que podem saturar o controlador.

Exemplo: WP com altitude 100m → 200m gera ~3 waypoints intermediários (100 → 133 → 167 → 200m). Os waypoints interpolados são usados na simulação, mas os gráficos mostram apenas os waypoints originais definidos pelo usuário.

### Dicas de uso

- **WP1 é fixo na origem** (0, 0, 100m) — posição inicial da aeronave
- **Espaçamento recomendado**: manter pelo menos 300-500m entre waypoints para uma navegação suave
- **R_accept**: aumentar para 150-200m se os waypoints estiverem próximos (~200m entre si)
- **Variação de altitude**: funciona bem até ±50m por trecho (a interpolação cuida do resto)
- Se a simulação terminar antes de atingir todos os WPs, aumente o tempo de simulação

## Missões disponíveis

Selecionar via variável `missao` no `inicializar.m`:

| Missão | Descrição |
|--------|-----------|
| 1 | Voo reto nivelado (teste de controle puro, sem transições de WP) |
| 2 | Triângulo com altitude constante (4 waypoints a 100m) |
| 3 | Triângulo com variação de altitude (100m → 150m → 70m → 100m) |

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
                |              Autopilot2                    |
                |  Altitude Hold --> Pitch Control --> Elev  |
                |  Velocity Control ----------------> Thr   |
                |  Heading --> Roll Control ---------> Ail   |
                |  Washout Guinada -----------------> Rud   |
                +-----------------+-------------------------+
                                  v
                          Saturações dos atuadores
                   Sat_Throttle, Sat_Elevator, Sat_Aileron, Sat_Rudder
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

| Malha | Variável | Tipo | P | I | D | N | Saturação | Anti-Windup |
|-------|----------|------|---|---|---|---|-----------|-------------|
| Altitude Hold | `C_alt` | PID | 0.08 | 0.02 | 0.0 | 20 | [-0.17, 0.26] | clamping |
| Pitch (Atitude) | `C_theta` | PID | 0.260 | 0.143 | 0.0 | 20 | [-0.4363, 0.4363] | clamping |
| SAS Arfagem | `Kq` | Ganho | 0.1 | - | - | - | - | - |
| Velocidade | `C_vel` | PID | 0.05 | 0.02 | 0.01 | 20 | [-0.49, 0.51] | clamping |

### Látero-direcional

| Malha | Variável | Tipo | P | I | D | N | Saturação | Anti-Windup |
|-------|----------|------|---|---|---|---|-----------|-------------|
| Roll (Bank Angle) | `C_phi` | PID | 10.0 | 0.0 | 0.0 | 20 | [-0.43, 0.43] | back-calculation |
| Heading | - | P | 2.5 | 0 | 0 | - | [-1.0, 1.0] | - |
| SAS Rolamento | `Kp` | Ganho | 0.119 | - | - | - | - | - |
| Heading → phi | - | Ganho | 0.3 | - | - | - | - | - |
| Amortecedor Guinada | `Kr` | Ganho + Washout | 0.15 | - | - | - | filtro s/(s+1) | - |

### Saturações dos atuadores

| Atuador | Limites | Unidade |
|---------|---------|---------|
| Sat_Throttle | [0, 1] | adimensional |
| Sat_Elevator | [-0.4363, 0.4363] | rad (±25°) |
| Sat_Aileron | [-0.4363, 0.4363] | rad (±25°) |
| Sat_Rudder | [-0.4363, 0.4363] | rad (±25°) |

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
