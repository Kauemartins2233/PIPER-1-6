# PIPER-1-6

Modelagem, simulacao e controle da aeronave **Piper J-3 Cub** em escala 1/6.

## Estrutura do repositorio

```
PIPER-1-6-main/
  inicializar.m       -- Script principal (carrega parametros, ganhos e waypoints)
  modelos/             -- Modelos matematicos (linear e nao linear 6-DOF)
    Linear/            -- Matrizes de espaco de estados (A, B, C, D)
    Nao Linear/        -- S-Function, dinamica, aerodinamica, propulsao
  trim/                -- Algoritmo de trimagem (equilibrio via fmincon)
  controle/            -- Malhas de controle PID (longitudinal e latero-direcional)
    Linear/            -- Controladores aplicados ao modelo linear
    Nao Linear/        -- Controladores aplicados ao modelo nao linear (modeloNL1.slx)
  guiagem/             -- Sistema de guiagem por waypoints (LOS + autopilot)
  Arduino/             -- Firmware para ESP32/Arduino (interface X-Plane via UDP)
  Xplane/              -- Integracao com X-Plane (em desenvolvimento)
```

## Aeronave

- **Modelo:** Piper J-3 Cub escala 1/6
- **Velocidade de cruzeiro:** 15 m/s
- **Altitude de equilibrio:** 100 m
- **12 estados:** u, v, w, p, q, r, phi, theta, psi, xN, xE, xD
- **4 entradas de controle:** throttle, profundor, aileron, leme
- **S-Function unica** (`sfunction_piper.m`): 21 outputs, usada por todos os modelos Simulink

## Como usar

### Simulacao de controle (modeloNL1.slx)

```matlab
>> inicializar                                         % carrega workspace
>> open('controle/Nao Linear/modeloNL1.slx')           % abre modelo
>> % Simular (Ctrl+T)
```

### Simulacao de guiagem (NL_guidance.slx)

```matlab
>> inicializar                                         % carrega workspace
>> open('guiagem/NL_guidance.slx')                     % abre modelo
>> setup_pid_blocks                                    % aplica PID completo (D, N)
>> % Simular (Ctrl+T)
>> plot3d_voo                                          % visualizar trajetoria 3D
```

### Trimagem (calculo de equilibrio)

```matlab
>> cd trim/
>> executar                                            % carrega parametros
>> trimagem_piper                                      % executa otimizacao via fmincon
```

## Dependencias

- MATLAB R2024a ou superior
- Simulink
- Optimization Toolbox (para trimagem)

Para detalhes de cada modulo, consultar o `readme.md` dentro de cada pasta.
