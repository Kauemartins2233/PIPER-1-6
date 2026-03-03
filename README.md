# PIPER-1-6

Modelagem, simulacao e controle da aeronave **Piper J-3 Cub** em escala 1/6.

## Estrutura do repositorio

```
PIPER-1-6-main/
  modelos/            -- Modelos matematicos (linear e nao linear 6-DOF)
    Linear/           -- Matrizes de espaco de estados (A, B, C, D)
    Nao Linear/       -- S-Function, dinamica, aerodinamica, propulsao
  trim/               -- Algoritmo de trimagem (equilibrio via fmincon)
  controle/           -- Malhas de controle PI (longitudinal e latero-direcional)
    Linear/           -- Controladores aplicados ao modelo linear
    Nao Linear/       -- Controladores aplicados ao modelo nao linear
  Arduino/            -- Firmware para ESP32/Arduino (interface X-Plane via UDP)
  Xplane/             -- Integracao com X-Plane (em desenvolvimento)
  guiagem/            -- Sistema de guiagem (em desenvolvimento)
```

## Aeronave

- **Modelo:** Piper J-3 Cub escala 1/6
- **Velocidade de cruzeiro:** 15 m/s
- **12 estados:** u, v, w, p, q, r, phi, theta, psi, xN, xE, xD
- **4 entradas de controle:** throttle, profundor, aileron, leme

## Como comecar

1. Abrir o MATLAB na pasta `modelos/Nao Linear/`
2. Rodar `executar.m` para carregar parametros e ponto de equilibrio
3. Abrir o modelo Simulink desejado (`modelo.slx` ou `modeloNL.slx`)
4. Rodar a simulacao

Para detalhes de cada modulo, consultar o `readme.md` dentro de cada pasta.
