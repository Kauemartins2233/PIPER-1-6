# X-Plane Integration

Integração do autopiloto PIPER-1-6 com o simulador X-Plane via plugin XPlaneConnect (XPC).

## Estrutura

```
Xplane/
├── teste.slx              # Modelo Simulink para comunicação com X-Plane
├── send_controls.m        # Função que envia comandos de controle via UDP
├── dados_da_missao.mat    # Dados da missão (waypoints, parâmetros)
├── XPlaneConnect/          # Plugin XPC compilado (instalar em X-Plane/Resources/plugins/)
│   ├── 64/
│   │   ├── win.xpl        # Plugin Windows
│   │   └── lin.xpl        # Plugin Linux
│   ├── win.xpl
│   ├── lin.xpl
│   └── mac.xpl
└── XPlaneConnect-master/   # Biblioteca XPC completa (código-fonte + API MATLAB)
    └── MATLAB/
        └── +XPlaneConnect/ # Funções MATLAB: openUDP, sendCTRL, getDREFs, etc.
```

## Como usar

### 1. Instalar o plugin no X-Plane

Copie a pasta `XPlaneConnect/` para `X-Plane/Resources/plugins/`.

### 2. Configurar o MATLAB

Adicione a pasta `XPlaneConnect-master/MATLAB/` ao path do MATLAB:

```matlab
addpath('Xplane/XPlaneConnect-master/MATLAB');
```

### 3. Executar

1. Abra o X-Plane com a aeronave Piper J-3 Cub
2. Execute `inicializar.m` no MATLAB para carregar os parâmetros
3. Abra e rode `teste.slx`

## Comunicação

- **Protocolo**: UDP na porta 49009 (localhost)
- **Envio de controle**: `send_controls.m` envia `[elevator, aileron, rudder, throttle]`
- **Conexão**: Variável global `GlobalSocket` compartilhada entre blocos de leitura e escrita
- **Valores não utilizados**: Preenchidos com `-998` (convenção XPC para "não alterar")

## Dependências

- MATLAB/Simulink R2025a
- X-Plane 11 ou 12
- Plugin XPlaneConnect (incluído neste repositório)
