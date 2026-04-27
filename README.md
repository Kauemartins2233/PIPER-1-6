# PIPER-1-6

Modelagem, simulação e controle da aeronave **Piper J-3 Cub** em escala 1/6.

## Aeronave

- Velocidade de cruzeiro: **15 m/s**
- Altitude de equilíbrio: **100 m**
- 12 estados: `u, v, w, p, q, r, phi, theta, psi, xN, xE, xD`
- 4 entradas: `throttle, profundor, aileron, leme`

## Estrutura do repositório

```
PIPER-1-6/
├── README.md                  ← este arquivo
├── inicializar.m              ← script de bootstrap (ganhos, trim, waypoints)
│
├── modelos/                   ← modelo matemático da planta
│   ├── Não Linear/            ←   S-Function 6-DOF + aerodinâmica + propulsão
│   └── Linear/                ←   matrizes A,B,C,D (linearização para análise)
│
├── trim/                      ← cálculo do estado de equilíbrio (fmincon)
├── controle/                  ← malhas PID em Simulink (não-HIL)
├── guiagem/                   ← waypoints + LOS + GUI visual
├── Arduino/                   ← Hardware-in-the-Loop com Arduino Mega
│   ├── HIL_simulink/          ←   v1: só controle embarcado
│   └── guiagem embarcada/     ←   v2: controle + guiagem embarcados
├── Xplane/                    ← integração X-Plane via XPC (UDP)
│
├── comparar_*.m               ← scripts de análise comparativa
├── imagens/                   ← figuras de apresentação
└── documentos relevantes/     ← dissertações de referência (PDFs)
```

## Quick-start por módulo

Cada caminho abaixo é independente — escolha o que você quer fazer.

### 1. Simulação não-linear pura (sem hardware)

```matlab
>> inicializar
>> open('controle/Não Linear/modeloNL1.slx')           % controle longitudinal+lateral
>> % Ctrl+T para simular
```

### 2. Guiagem por waypoints (com GUI)

```matlab
>> inicializar
>> gui_waypoints                                       % clique no mapa, depois SIMULAR
```

Ou via linha de comando:

```matlab
>> open('guiagem/NL_guidance.slx')
>> % Ctrl+T
>> plot3d_voo                                          % trajetória 3D
```

### 3. Trimagem (recálculo do equilíbrio)

```matlab
>> cd trim/
>> executar
>> trimagem_piper                                      % otimização fmincon
```

### 4. Hardware-in-the-Loop (HIL) — Arduino Mega

Pré-requisito: gravar
[Arduino/HIL_simulink/arduino_controlador_manual](Arduino/HIL_simulink/arduino_controlador_manual)
no Mega 2560 via Arduino IDE. Ver porta COM em
`Gerenciador de Dispositivos`; ajustar
[Arduino/HIL_simulink/hil_serial_step.m](Arduino/HIL_simulink/hil_serial_step.m):19
se diferente de COM4.

```matlab
>> cd Arduino/HIL_simulink
>> ping_arduino           % sanity check do link
>> run_hil_test           % sim no trim (~50s)
>> run_hil_manobra        % 3 cenários transientes + figura PNG
>> clear hil_serial_step  % fecha hil_log.csv depois da sim
>> view_hil_log           % plota o tráfego serial registrado
```

Detalhes completos em [Arduino/README.md](Arduino/README.md).

### 5. Integração X-Plane

```matlab
>> inicializar_xplane
>> open('Xplane/xplane_autopilot.slx')
```

Detalhes em [Xplane/readme.md](Xplane/readme.md).

## Dependências

- MATLAB R2024a ou superior (testado em R2025b)
- Simulink
- Optimization Toolbox (para trimagem)
- Arduino IDE 2.x (para HIL)

## Convenções

- **S-Function única:** `modelos/Não Linear/sfunction_piper.m` é a planta usada por
  TODOS os modelos Simulink (`modeloNL1.slx`, `NL_guidance.slx`, `modeloNL1_HIL.slx`,
  `modelo_HIL_guiagem.slx`). 21 saídas.
- **Workspace base:** `inicializar.m` faz `clear; clc` no início e popula o base
  workspace. Não use variáveis suas com nomes que conflitem (`Xe`, `Ue`, `WPs`, etc.).
- **Trim:** `Xe(12) = -100` significa altitude = +100 m (sistema NED).

Para detalhes de cada módulo, ver o `readme.md` correspondente dentro da pasta.
