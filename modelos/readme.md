# Modelos - Piper J-3 Cub 1/6

Modelos matematicos da aeronave utilizados para simulacao e projeto de controladores.

## Subdiretorios

| Pasta | Descricao |
|-------|-----------|
| `Linear/` | Matrizes de espaco de estados (A, B, C, D) obtidas por linearizacao |
| `Nao Linear/` | Modelo 6-DOF completo via S-Function do Simulink (21 saidas) |

## Funcoes compartilhadas

As funcoes do modelo nao linear (`sfunction_piper.m`, `dyn_rigidbody.m`, `aerodynamics.m`, `propulsion.m`, `equilibrium.m`, etc.) ficam em `Nao Linear/` e sao referenciadas por outros modulos (trim, controle, guiagem) via `addpath`.

O script `inicializar.m` (na raiz do repositorio) adiciona automaticamente `modelos/Nao Linear/` ao path do MATLAB.
