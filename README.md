# PIPER-1-6

Modelagem, simulação e controle da aeronave **Piper J-3 Cub** em escala 1/6.

## Estrutura do repositório

```
PIPER-1-6/
  inicializar.m       -- Script principal (carrega parâmetros, ganhos e waypoints)
  modelos/             -- Modelos matemáticos (linear e não linear 6-DOF)
    Linear/            -- Matrizes de espaço de estados (A, B, C, D)
    Não Linear/        -- S-Function, dinâmica, aerodinâmica, propulsão
  trim/                -- Algoritmo de trimagem (equilíbrio via fmincon)
  controle/            -- Malhas de controle PID (longitudinal e látero-direcional)
    Linear/            -- Controladores aplicados ao modelo linear
    Não Linear/        -- Controladores aplicados ao modelo não linear (modeloNL1.slx)
  guiagem/             -- Sistema de guiagem por waypoints (LOS + autopilot)
  Arduino/             -- Firmware para ESP32/Arduino (interface X-Plane via UDP)
  Xplane/              -- Integração com X-Plane (em desenvolvimento)
```

## Aeronave

- **Modelo:** Piper J-3 Cub escala 1/6
- **Velocidade de cruzeiro:** 15 m/s
- **Altitude de equilíbrio:** 100 m
- **12 estados:** u, v, w, p, q, r, phi, theta, psi, xN, xE, xD
- **4 entradas de controle:** throttle, profundor, aileron, leme
- **S-Function única** (`sfunction_piper.m`): 21 outputs, usada por todos os modelos Simulink

## Como usar

### Simulação de controle (modeloNL1.slx)

```matlab
>> inicializar                                         % carrega workspace
>> open('controle/Não Linear/modeloNL1.slx')           % abre modelo
>> % Simular (Ctrl+T)
```

### Simulação de guiagem — Interface visual (recomendado)

```matlab
>> gui_waypoints                                       % abre interface visual
```

Clique no mapa para posicionar waypoints, ajuste altitude e velocidade, e clique **SIMULAR**.

### Simulação de guiagem — Linha de comando

```matlab
>> inicializar                                         % carrega workspace
>> open('guiagem/NL_guidance.slx')                     % abre modelo
>> % Simular (Ctrl+T)
>> plot3d_voo                                          % visualizar trajetória 3D
```

### Trimagem (cálculo de equilíbrio)

```matlab
>> cd trim/
>> executar                                            % carrega parâmetros
>> trimagem_piper                                      % executa otimização via fmincon
```

## Dependências

- MATLAB R2024a ou superior
- Simulink
- Optimization Toolbox (para trimagem)

Para detalhes de cada módulo, consultar o `readme.md` dentro de cada pasta.
