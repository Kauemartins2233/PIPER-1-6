# X-Plane Integration

Integracao do autopiloto PIPER-1-6 com o simulador X-Plane via plugin XPlaneConnect (XPC).

## Estrutura

```
Xplane/
├── xplane_autopilot.slx       # Modelo Simulink - autopiloto em malha fechada com X-Plane
├── criar_modelo_xplane.m       # Script que gera o xplane_autopilot.slx programaticamente
├── inicializar_xplane.m        # Inicializacao (ganhos PID + paths XPC)
├── read_xplane.m               # Le sensores do X-Plane via getDREFs (10 sinais)
├── send_xplane.m               # Envia comandos de controle (rad -> normalizado)
├── close_xplane.m              # Fecha conexao UDP (chamado automaticamente ao parar)
├── teste.slx                   # Modelo antigo (apenas envio de controle)
├── send_controls.m             # Funcao antiga de envio
├── dados_da_missao.mat         # Dados da missao (waypoints)
├── XPlaneConnect/              # Plugin XPC compilado (instalar no X-Plane)
│   ├── 64/
│   │   ├── win.xpl            # Plugin Windows 64-bit
│   │   └── lin.xpl            # Plugin Linux 64-bit
│   ├── win.xpl                # Plugin Windows 32-bit (X-Plane 9)
│   ├── lin.xpl
│   └── mac.xpl
└── XPlaneConnect-master/       # Biblioteca XPC completa (API MATLAB)
    └── MATLAB/
        └── +XPlaneConnect/     # Funcoes: openUDP, sendCTRL, getDREFs, etc.
```

## Como usar

### 1. Instalar o plugin no X-Plane

Copie a pasta `XPlaneConnect/` para `X-Plane/Resources/plugins/`.

**X-Plane 9**: Use o plugin `win.xpl` da pasta raiz (32-bit).
**X-Plane 11/12**: Use o plugin da pasta `64/`.

### 2. Gerar o modelo Simulink (apenas na primeira vez)

```matlab
cd('caminho/para/PIPER-1-6-GUI')
inicializar_xplane
criar_modelo_xplane
```

Isso cria o arquivo `xplane_autopilot.slx` com todo o autopiloto montado.

### 3. Executar

```matlab
% 1. Carregar workspace
inicializar_xplane

% 2. Abrir modelo
open('Xplane/xplane_autopilot.slx')

% 3. Iniciar X-Plane com Piper J-3 Cub
%    (posicionar a aeronave em voo a ~100m de altitude)

% 4. Simular no Simulink (Ctrl+T)
%    O modelo roda em tempo real ate voce parar (Ctrl+C)
```

### 4. Alterar referencias

No workspace do MATLAB, antes de simular:

```matlab
h_ref   = 150;    % altitude desejada (m)
VT_ref  = 18;     % velocidade desejada (m/s)
psi_ref = pi/4;   % proa desejada (rad, 45 graus)
```

## Arquitetura do Autopiloto

```
X-Plane 9 (Planta real)
    |         ^
    | getDREFs|  sendCTRL
    v         |
read_xplane   send_xplane
    |         ^
    | [deg->rad]  [rad->norm]
    v         |
   AUTOPILOT PIDs
   (mesmos ganhos do modeloNL1.slx)
```

### Sinais lidos do X-Plane (read_xplane.m)

| # | Sinal | DataRef | Conversao |
|---|-------|---------|-----------|
| 1 | VT (m/s) | `true_airspeed` | nenhuma |
| 2 | theta (rad) | `theta` | deg->rad |
| 3 | q (rad/s) | `Q` | deg/s->rad/s |
| 4 | h (m) | `elevation` | nenhuma |
| 5 | phi (rad) | `phi` | deg->rad |
| 6 | p (rad/s) | `P` | deg/s->rad/s |
| 7 | psi (rad) | `psi` | deg->rad |
| 8 | r (rad/s) | `R` | deg/s->rad/s |
| 9 | xN (m) | `-local_z` | relativo ao inicio |
| 10 | xE (m) | `local_x` | relativo ao inicio |

### Comandos enviados ao X-Plane (send_xplane.m)

| # | Comando | Faixa autopiloto | Faixa X-Plane | Conversao |
|---|---------|-----------------|---------------|-----------|
| 1 | Elevator | [-0.4363, +0.4363] rad | [-1, +1] | / 0.4363 |
| 2 | Aileron | [-0.4363, +0.4363] rad | [-1, +1] | / 0.4363 |
| 3 | Rudder | [-0.4363, +0.4363] rad | [-1, +1] | / 0.4363 |
| 4 | Throttle | [0, 1] | [0, 1] | nenhuma |

## Malhas de controle

- **Altitude**: h_ref -> PID_alt -> theta_ref (sat: [-0.17, 0.26] rad)
- **Arfagem**: theta_ref -> PID_theta - Kq*q -> delta_e (sat: +/-25 deg)
- **Velocidade**: VT_ref -> PID_vel + trim -> delta_T (sat: [0, 1])
- **Heading**: psi_ref -> ganho 0.3 -> phi_ref
- **Rolamento**: phi_ref -> PID_phi - Kp_sas*p -> delta_a (sat: +/-25 deg)
- **Guinada**: r -> washout s/(s+1) -> Kr -> delta_r (sat: +/-25 deg)

## Comunicacao

- **Protocolo**: UDP na porta 49009 (localhost)
- **Conexao**: Variavel global `GlobalSocket` compartilhada entre read e send
- **Valores nao utilizados**: Preenchidos com `-998` (convencao XPC)
- **Sample time**: 0.05 s (20 Hz) - ajustavel via `Ts_xplane`

## Solucao de problemas

### Elevator invertido
Se a aeronave cabrar em vez de picar (ou vice-versa), negar o sinal do elevator
em `send_xplane.m`: trocar `elevator = u(1) / max_deflection` por
`elevator = -u(1) / max_deflection`.

### DataRef nao encontrado (X-Plane 9)
Se `true_airspeed` nao existir, substituir por `indicated_airspeed` em
`read_xplane.m`.

### Aeronave oscila muito
Reduzir ganhos em `inicializar.m` (comece por `C_theta.Kp` e `C_phi.Kp`)
ou aumentar `Ts_xplane` para 0.1 s.

## Dependencias

- MATLAB/Simulink R2025a
- X-Plane 9 (compativel tambem com 11/12)
- Plugin XPlaneConnect (incluido neste repositorio)
