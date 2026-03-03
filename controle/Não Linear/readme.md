# Controle Nao Linear - Piper J-3 Cub 1/6

Modelo Simulink (`modeloNL.slx`) que aplica as malhas de controle longitudinal e latero-direcional sobre o **modelo nao linear (6DOF)** da aeronave.

## Descricao

Neste modelo, a planta e o modelo nao linear completo do Piper 1/6 (S-Function `sfunction_piper`, mesma da pasta `modelos/Nao Linear/`). As mesmas malhas PI projetadas sobre o modelo linear sao aplicadas aqui para verificar se o controlador mantém desempenho aceitavel quando a planta possui nao linearidades, acoplamento entre modos e efeitos nao modelados.

Os resultados mostraram que, para manobras suaves, as respostas do modelo nao linear sao praticamente sobrepostas as do modelo linear, validando o projeto dos controladores.

## Arquivos

| Arquivo | Descricao |
|---------|-----------|
| `modeloNL.slx` | Modelo Simulink com planta nao linear (S-Function) + malhas de controle |
| `modeloNL.slxc` | Cache compilado do modelo (gerado automaticamente) |

## Dependencias

Antes de rodar, e necessario ter no workspace:
- `par_aero`, `par_prop`, `par_gen` (structs de parametros da aeronave)
- `Xe` (vetor de estado de equilibrio, 12x1)
- `INPUTS` (vetor de comandos de equilibrio)

Esses dados sao carregados rodando `modelos/Nao Linear/executar.m` ou o `.mat` correspondente + `equilibrium.m`.

## Como rodar

1. Carregar os parametros no workspace (rodar `modelos/Nao Linear/executar.m`).
2. Abrir `modeloNL.slx` no Simulink.
3. Definir as referencias desejadas (velocidade, altitude, heading).
4. No bloco de entrada, garantir que `Ue` esta configurado como `INPUTS(1:4)`.
5. Rodar a simulacao. Resultados ficam em `out.Y`.
6. Para visualizacao, usar `modelos/Nao Linear/plot_long.m` (longitudinal) ou `gerar_log.m` (latero-direcional).

## Ponto de equilibrio

```
Xe = [15; 0; -1.8343; 0; 0; 0; 0; -0.1217; 0; 0; 0; -100]
Ue = INPUTS(1:4)
```

## Resultados

### Longitudinal
- Altitude: erro maximo < 0.03 m em relacao a referencia
- Velocidade (VT): estabiliza em ~14.89 m/s (erro ~0.11 m/s para ref de 15 m/s; corrigivel aumentando Ki)
- Theta: estabiliza em valor ligeiramente acima do trim apos transiente

### Latero-direcional
- Phi e Psi: respostas praticamente identicas ao modelo linear
- Curva coordenada: excursao transitoria em phi (~7.8 graus) gera taxa de giro, aeronave nivela e estabiliza na nova proa
- Erro estatico desprezivel em ambas as variaveis

## Ganhos dos controladores (valores atuais)

### Longitudinal

| Malha | Tipo | P | I | D | N | Sat |
|-------|------|---|---|---|---|-----|
| Altitude Hold | PID | 0.596 | 0.356 | -0.0142 | 6.17 | [-0.17, 0.26] |
| Pitch (Atitude) | PID | 20.31 | 22.60 | 1.767 | 1159.4 | sem |
| SAS Arfagem | Ganho | Kq = 0.1 | | | | |
| Velocidade | PID | -0.528 | -0.431 | -0.0363 | 11348.4 | sem |

### Latero-direcional

| Malha | Tipo | P | I | D | N | Sat |
|-------|------|---|---|---|---|-----|
| Roll (Bank Angle) | PID | 26.79 | 13.17 | -0.0876 | 305.9 | sem |
| SAS Rolamento | Ganho | Kp = 0.119 | | | | |
| Heading -> phi | Ganho | 0.8 | | | | |
| Amortecedor Guinada | Ganho + Washout | Kr = 0.1, filtro s/(s+1) | | | | |

Ver diagramas de blocos completos em `controle/readme.md`.
