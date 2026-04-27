# Modelos - Piper J-3 Cub 1/6

Modelos matemáticos da aeronave utilizados para simulação e projeto de controladores.

## Subdiretórios

| Pasta | Descrição |
|-------|-----------|
| `Linear/` | Matrizes de espaço de estados (A, B, C, D) obtidas por linearização |
| `Não Linear/` | Modelo 6-DOF completo via S-Function do Simulink (21 saídas) |

## Funções compartilhadas

As funções do modelo não linear (`sfunction_piper.m`, `dyn_rigidbody.m`, `aerodynamics.m`, `propulsion.m`, `equilibrium.m`, etc.) ficam em `Não Linear/` e são referenciadas por outros módulos (trim, controle, guiagem) via `addpath`.

O script `inicializar.m` (na raiz do repositório) adiciona automaticamente `modelos/Não Linear/` ao path do MATLAB.
