# Modelo Não Linear - Piper J-3 Cub 1/6

Modelo dinâmico não linear (6DOF) da aeronave Piper J-3 Cub em escala 1/6, integrado ao Simulink via S-Function.

## Arquivos

| Arquivo | Descrição |
|---------|-----------|
| `sfunction_piper.m` | S-Function Level-1 do Simulink (inicialização, derivadas e saídas) |
| `dyn_rigidbody.m` | Derivadas dos 12 estados (equações de corpo rígido) |
| `obs_rigidbody.m` | Mapeamento estado -> saídas observáveis |
| `aerodynamics.m` / `aerodynamics2.m` | Forças e momentos aerodinâmicos |
| `propulsion.m` | Modelo de propulsão |
| `equilibrium.m` | Condição de equilíbrio (Xe, Ue) |
| `decoupling.m` | Desacoplamento longitudinal / látero-direcional e geração de (A,B,C,D) |
| `lin.m` | Linearização em torno do ponto de equilíbrio |
| `ISA.m` | Atmosfera padrão internacional |
| `modelo.slx` | Modelo Simulink não linear (planta aberta, sem controlador) |
| `gerar_log.m` | Extrai variáveis látero-direcionais de `out.Y` e gera log para OEM |
| `plot_long.m` | Extrai e plota variáveis longitudinais de `out.Y` |
| `executar.m` | Script de inicialização básica (carrega .mat e chama `equilibrium`) |

## S-Function: sfunction_piper

**Arquivo único** usado por todos os modelos Simulink do repositório (modeloNL1.slx, NL_guidance.slx, q2_trim_modelo24a.slx).

- **12 estados contínuos:** u, v, w, p, q, r, phi, theta, psi, xN, xE, xD
- **4 entradas:** delta_T (throttle), delta_e (profundor), delta_a (aileron), delta_r (leme)
- **21 saídas:**

| Índice | Saída | Descrição |
|--------|-------|-----------|
| 1 | VT | Velocidade total |
| 2 | alpha | Ângulo de ataque |
| 3 | beta | Ângulo de derrapagem |
| 4-6 | p, q, r | Taxas angulares |
| 7-9 | phi, theta, psi | Ângulos de Euler |
| 10-11 | xN, xE | Posição NED (Norte, Leste) |
| 12 | -xD | Altitude (positiva para cima) |
| 13-15 | u, v, w | Velocidades no corpo |
| 16-21 | up, vp, wp, pp, qp, rp | Derivadas (monitoramento) |

- **DirFeedthrough = 0:** Elimina loops algébricos. Os outputs 16-21 (derivadas) usam `u` do passo anterior, o que é aceitável para monitoramento.

## Ponto de equilíbrio

Definido em `equilibrium.m` (valores obtidos via `trimagem_piper.m` com fmincon):

```
Xe = [15; 0; -1.8343; 0; 0; 0; 0; -0.1217; 0; 0; 0; -100]
Ue = [0.4914; 0.0155; 0; 0]
```

Condição: VT = 15 m/s, theta = -0.1217 rad, altitude = 100 m, voo reto nivelado.

## Passo a passo

1. Na raiz do repositório, rodar `inicializar` (carrega parâmetros, ganhos e equilíbrio).
2. Abrir o modelo Simulink desejado.
3. Rodar a simulação. Resultados ficam em `out`.
4. Para visualização:
   - `gerar_log.m` -- variáveis látero-direcionais e trajetória 3D
   - `plot_long.m` -- variáveis longitudinais
