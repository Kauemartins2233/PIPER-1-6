# Modelo Nao Linear - Piper J-3 Cub 1/4

Modelo dinamico nao linear (6DOF) da aeronave Piper J-3 Cub em escala 1/4, integrado ao Simulink via S-Function.

## Arquivos

| Arquivo | Descricao |
|---------|-----------|
| `sfunction_piper.m` | S-Function do Simulink (inicializacao, derivadas e saidas) |
| `dyn_rigidbody.m` | Derivadas dos 12 estados (equacoes de corpo rigido) |
| `obs_rigidbody.m` | Mapeamento estado -> saidas observaveis |
| `aerodynamics.m` / `aerodynamics2.m` | Forcas e momentos aerodinamicos |
| `propulsion.m` | Modelo de propulsao |
| `equilibrium.m` | Condicao de equilibrio (Xe, Ue) |
| `decoupling.m` | Desacoplamento longitudinal / latero-direcional e geracao de (A,B,C,D) |
| `lin.m` | Linearizacao em torno do ponto de equilibrio |
| `ISA.m` | Atmosfera padrao internacional |
| `modelo.slx` | Modelo Simulink nao linear |
| `gerar_log.m` | Extrai variaveis latero-direcionais de `out.Y` e gera log para OEM |
| `plot_long.m` | Extrai e plota variaveis longitudinais de `out.Y` |
| `executar.m` | Script de inicializacao (carrega .mat e chama `equilibrium`) |

## Estados, entradas e saidas

- **12 estados:** u, v, w, p, q, r, phi, theta, psi, xN, xE, xD
- **4 entradas:** delta_T (throttle), delta_e (profundor), delta_a (aileron), delta_r (leme)
- **15 saidas:** VT, alpha, beta, p, q, r, phi, theta, psi, xN, xE, -xD, pdot, rdot, ay

## Passo a passo

1. Carregar os dados da aeronave no workspace. Rode `trim_vector.mat` junto aos arquivos extraidos do zip do modelo (ou rode `executar.m`, que carrega `Sato_longitudinal_Piper_1_6.mat` e chama `equilibrium`). Isso disponibiliza no workspace:
   - `par_aero`, `par_prop`, `par_gen` (structs com parametros aerodinamicos, propulsivos e geometricos)
   - `Xe` (ponto de equilibrio): `[15; 0; -1.8343; 0; 0; 0; 0; -0.1217; 0; 0; 0; -100]`
   - `INPUTS` (vetor de comandos iniciais referente ao equilibrio)

2. Abrir `modelo.slx` no Simulink. Antes de rodar:
   - Definir a manobra desejada (alterar ganhos ou tipo de comando: pulse, doublet, etc.)
   - No bloco de entrada, alterar `Ue` para `INPUTS(1:4)`

3. Rodar a simulacao. Os resultados ficam em `out.Y` no workspace.

4. Para visualizar:
   - `gerar_log.m` -- variaveis latero-direcionais (beta, p, r, phi, pdot, rdot, ay) e trajetoria 3D
   - `plot_long.m` -- variaveis longitudinais (q, theta, VT) e trajetoria 3D

## Ponto de equilibrio

Definido em `equilibrium.m`:

```
Xe = [U; V; W; 0; 0; 0; 0; Theta; 0; 0; 0; -560]
Ue = [0.05; -1.17391; 0; 0]
```

Com VT = 15 m/s, Alpha = 0.29129 rad (AVL), altitude ~560 m (moda do log de voo).
