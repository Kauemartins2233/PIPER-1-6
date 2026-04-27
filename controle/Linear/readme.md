# Controle Linear - Piper J-3 Cub 1/6

Modelo Simulink (`modelo_linear.slx`) que aplica as malhas de controle longitudinal e látero-direcional sobre o **modelo linearizado** da aeronave.

## Descrição

Neste modelo, a planta é representada pelas matrizes de espaço de estados (A, B, C, D) dos modos longitudinal e látero-direcional obtidas na etapa de linearização (ver `modelos/Linear/MATRIZES.m`). As malhas de controle PID projetadas via PID Tuner atuam sobre essa planta linear, servindo como caso de referência para comparação com o modelo não linear.

## Arquivos

| Arquivo | Descrição |
|---------|-----------|
| `modelo_linear.slx` | Modelo Simulink com planta linear + malhas de controle |
| `modelo_linear.slxc` | Cache compilado do modelo (gerado automaticamente) |

## Como rodar

1. Na raiz do repositório, rodar `inicializar` (carrega parâmetros, ganhos e equilíbrio).
2. Abrir `controle/Linear/modelo_linear.slx` no Simulink.
3. Definir as referências desejadas (velocidade, altitude, heading).
4. Rodar a simulação. Os resultados ficam em `out.Y`.

```matlab
>> inicializar
>> open('controle/Linear/modelo_linear.slx')
>> % Simular (Ctrl+T)
```

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

### Látero-direcional (estados: [V, p, r, phi, psi], entradas: [delta_a, delta_r])

```
A_lat = [-0.3414 -1.658  -14.75  9.734  0;
         -1.298  -28.3    7.924  0      0;
          0.9711 -2.081  -1.556  0      0;
          0       1      -0.1223 0      0;
          0       0       1.007  0      0]
B_lat = [0 0.0378; 3.296 0.0085; 0.171 -0.2389; 0 0; 0 0]
```

## Pontos de equilíbrio (condição inicial)

```
Xe_long = [15, -1.8343, 0, -0.1217, -100]
Xe_lat  = [0, 0, 0, 0, 0]
```

## Ganhos dos controladores (valores atuais no modelo)

### Longitudinal

| Malha | Tipo | P | I | D | N | Saturação | Anti-Windup |
|-------|------|---|---|---|---|-----------|-------------|
| Altitude Hold | PID | 0.08 | 0.02 | 0.0 | 20 | [-0.17, 0.26] | clamping |
| Pitch (Atitude) | PID | 0.260 | 0.143 | 0.0 | 20 | [-0.4363, 0.4363] | clamping |
| SAS Arfagem | Ganho | Kq = 0.1 | | | | | |
| Velocidade | PID | 0.05 | 0.02 | 0.01 | 20 | [-0.49, 0.51] | clamping |

### Látero-direcional

| Malha | Tipo | P | I | D | N | Saturação | Anti-Windup |
|-------|------|---|---|---|---|-----------|-------------|
| Roll (Bank Angle) | PID | 10.0 | 0.0 | 0.0 | 20 | [-0.43, 0.43] | back-calculation |
| SAS Rolamento | Ganho | Kp = 0.119 | | | | | |
| Heading → phi | Ganho | 0.3 | | | | | |
| Amortecedor Guinada | Ganho + Washout | Kr = 0.15, filtro s/(s+1) | | | | | |

### Saturações dos atuadores

| Atuador | Limites | Unidade |
|---------|---------|---------|
| Sat_Throttle | [0, 1] | adimensional |
| Sat_Elevator | [-0.4363, 0.4363] | rad (±25°) |
| Sat_Aileron | [-0.4363, 0.4363] | rad (±25°) |
| Sat_Rudder | [-0.4363, 0.4363] | rad (±25°) |

Ver diagramas de blocos completos em `controle/readme.md`.
