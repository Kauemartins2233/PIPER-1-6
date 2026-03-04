# Modelo Nao Linear - Piper J-3 Cub 1/6

Modelo dinamico nao linear (6DOF) da aeronave Piper J-3 Cub em escala 1/6, integrado ao Simulink via S-Function.

## Arquivos

| Arquivo | Descricao |
|---------|-----------|
| `sfunction_piper.m` | S-Function Level-1 do Simulink (inicializacao, derivadas e saidas) |
| `dyn_rigidbody.m` | Derivadas dos 12 estados (equacoes de corpo rigido) |
| `obs_rigidbody.m` | Mapeamento estado -> saidas observaveis |
| `aerodynamics.m` / `aerodynamics2.m` | Forcas e momentos aerodinamicos |
| `propulsion.m` | Modelo de propulsao |
| `equilibrium.m` | Condicao de equilibrio (Xe, Ue) |
| `decoupling.m` | Desacoplamento longitudinal / latero-direcional e geracao de (A,B,C,D) |
| `lin.m` | Linearizacao em torno do ponto de equilibrio |
| `ISA.m` | Atmosfera padrao internacional |
| `modelo.slx` | Modelo Simulink nao linear (planta aberta, sem controlador) |
| `gerar_log.m` | Extrai variaveis latero-direcionais de `out.Y` e gera log para OEM |
| `plot_long.m` | Extrai e plota variaveis longitudinais de `out.Y` |
| `executar.m` | Script de inicializacao basica (carrega .mat e chama `equilibrium`) |

## S-Function: sfunction_piper

**Arquivo unico** usado por todos os modelos Simulink do repositorio (modeloNL1.slx, NL_guidance.slx, q2_trim_modelo24a.slx).

- **12 estados continuos:** u, v, w, p, q, r, phi, theta, psi, xN, xE, xD
- **4 entradas:** delta_T (throttle), delta_e (profundor), delta_a (aileron), delta_r (leme)
- **21 saidas:**

| Indice | Saida | Descricao |
|--------|-------|-----------|
| 1 | VT | Velocidade total |
| 2 | alpha | Angulo de ataque |
| 3 | beta | Angulo de derrapagem |
| 4-6 | p, q, r | Taxas angulares |
| 7-9 | phi, theta, psi | Angulos de Euler |
| 10-11 | xN, xE | Posicao NED (Norte, Leste) |
| 12 | -xD | Altitude (positiva para cima) |
| 13-15 | u, v, w | Velocidades no corpo |
| 16-21 | up, vp, wp, pp, qp, rp | Derivadas (monitoramento) |

- **DirFeedthrough = 0:** Elimina loops algebricos. Os outputs 16-21 (derivadas) usam `u` do passo anterior, o que e aceitavel para monitoramento.

## Ponto de equilibrio

Definido em `equilibrium.m` (valores obtidos via `trimagem_piper.m` com fmincon):

```
Xe = [15; 0; -1.8343; 0; 0; 0; 0; -0.1217; 0; 0; 0; -100]
Ue = [0.4914; 0.0155; 0; 0]
```

Condicao: VT = 15 m/s, theta = -0.1217 rad, altitude = 100 m, voo reto nivelado.

## Passo a passo

1. Na raiz do repositorio, rodar `inicializar` (carrega parametros, ganhos e equilibrio).
2. Abrir o modelo Simulink desejado.
3. Rodar a simulacao. Resultados ficam em `out`.
4. Para visualizacao:
   - `gerar_log.m` -- variaveis latero-direcionais e trajetoria 3D
   - `plot_long.m` -- variaveis longitudinais
