# Controle Linear - Piper J-3 Cub 1/6

Modelo Simulink (`modelo_linear.slx`) que aplica as malhas de controle longitudinal e latero-direcional sobre o **modelo linearizado** da aeronave.

## Descricao

Neste modelo, a planta e representada pelas matrizes de espaco de estados (A, B, C, D) dos modos longitudinal e latero-direcional obtidas na etapa de linearizacao (ver `modelos/Linear/MATRIZES.m`). As malhas de controle PI projetadas via PID Tuner atuam sobre essa planta linear, servindo como caso de referencia para comparacao com o modelo nao linear.

## Arquivos

| Arquivo | Descricao |
|---------|-----------|
| `modelo_linear.slx` | Modelo Simulink com planta linear + malhas de controle |
| `modelo_linear.slxc` | Cache compilado do modelo (gerado automaticamente) |

## Como rodar

1. Gerar as matrizes no workspace rodando `modelos/Linear/MATRIZES.m` (A_long, B_long, C_long, D_long, A_lat, B_lat, C_lat, D_lat, Xe_long, Xe_lat).
2. Abrir `modelo_linear.slx` no Simulink. Os blocos de espaco de estados leem automaticamente as matrizes do workspace.
3. Definir as referencias desejadas (velocidade, altitude, heading).
4. Rodar a simulacao. Os resultados ficam em `out.Y`.

## Matrizes utilizadas

### Longitudinal (estados: [u, w, q, theta, h], entradas: [delta_T, delta_e])

```
A_long = [-0.7475 -0.2987  0.87   -9.734  -0.0001677;
          -2.005  -5.782   9.65    1.19   -0.0009366;
          -0.2089 -1.708  -20.32   0      -1.166e-10;
           0       0       1       0       0;
           0.1214  0.9926  0      -15.11   0]
B_long = [5.62 0; 0 0; 0 163.1; 0 0; 0 0]
```

### Latero-direcional (estados: [V, p, r, phi, psi], entradas: [delta_a, delta_r])

```
A_lat = [-0.3414 -1.658  -14.75  9.734  0;
         -1.298  -28.3    7.924  0      0;
          0.9711 -2.081  -1.556  0      0;
          0       1      -0.1223 0      0;
          0       0       1.007  0      0]
B_lat = [0 0.0378; 3.296 0.0085; 0.171 -0.2389; 0 0; 0 0]
```

## Pontos de equilibrio (condicao inicial)

```
Xe_long = [15, -1.8343, 0, -0.1217, -100]
Xe_lat  = [0, 0, 0, 0, 0]
```

## Ganhos dos controladores (valores atuais no modelo)

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
