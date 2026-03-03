# Modelos - Piper J-3 Cub 1/6

Modelos matematicos da aeronave utilizados para simulacao e projeto de controladores.

## Subdiretorios

| Pasta | Descricao |
|-------|-----------|
| `Linear/` | Matrizes de espaco de estados (A, B, C, D) obtidas por linearizacao |
| `Não Linear/` | Modelo 6-DOF completo via S-Function do Simulink |

## Funcoes compartilhadas

As funcoes do modelo nao linear (`dyn_rigidbody.m`, `aerodynamics.m`, `propulsion.m`, etc.) ficam em `Não Linear/` e sao referenciadas por outros modulos (trim, controle) via `addpath`.
