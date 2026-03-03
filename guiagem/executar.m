%% executar.m - Inicializacao do modelo de guiagem NL_guidance.slx
% Carrega parametros da aeronave, define ganhos dos controladores PI,
% waypoints e condicao de equilibrio.
%
% Uso: Rodar este script antes de abrir/simular NL_guidance.slx
%
% NOTA: O modelo usa sfunction_piper com 21 outputs (mesma versao do trim).
%       Certifique-se de que trim/ esta no path ou altere o bloco S-Function
%       para usar sfunction_piper_trim.

clear; clc;

%% ========== Paths ==========
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, '..', 'modelos', 'Não Linear'));  % aerodynamics, dyn_rigidbody, etc.
addpath(fullfile(rootDir, '..', 'trim'));                    % sfunction_piper_trim (21 outputs)

%% ========== Parametros da Aeronave ==========
% Carrega par_aero, par_prop, par_gen
load('Sato_longitudinal_Piper_1_6.mat');

%% ========== Estado de Equilibrio ==========
equilibrium;  % Define Xe (12x1) e Ue (4x1)

% TrimInput usado pelos blocos Constant no Autopilot1
TrimInput = Ue';  % [delta_T, delta_e, delta_a, delta_r]

%% ========== Ganhos do Piloto Automatico ==========
% Valores extraidos do modelo nao linear (controle/Nao Linear/modeloNL.slx).
% Os blocos PI no modelo de guiagem usam apenas Kp e Ki (D=0, N=100).
% Os termos D sao documentados aqui para referencia caso se queira
% converter os blocos PI para PID completo.

% --- Longitudinal ---

% Altitude Hold: h_ref -> theta_ref (PID no modeloNL)
C_alt.Kp = 0.596304245000559;
C_alt.Ki = 0.356254495459697;
C_alt.Kd = -0.0141697733728916;   % D (nao usado no bloco PI da guiagem)
C_alt.N  = 6.17157424867561;      % N (nao usado no bloco PI da guiagem)
% Saturacao: [-0.17, 0.26] rad (hardcoded no modelo)

% Pitch (Atitude): theta_ref -> delta_e (PID no modeloNL)
C_theta.Kp = 20.3142831421082;
C_theta.Ki = 22.5973845921282;
C_theta.Kd = 1.76724278063426;    % D
C_theta.N  = 1159.39165466191;    % N

% Velocidade: VT_ref -> delta_T
% NOTA: No modeloNL existem 2 PIDs em cascata para velocidade.
% O modelo de guiagem tem apenas 1 bloco PI. Aqui usamos o PID2
% (malha externa). Para maior fidelidade, considerar reestruturar
% o bloco Velocity Control na guiagem.
%   PID1 (interno): P=-0.0486, I=-0.0042, D=-0.1245, N=1697.4
%   PID2 (externo): P=-0.0787, I=-0.0652, D=-0.0152, N=77.0
C_vel.Kp = -0.0786752433250596;
C_vel.Ki = -0.0651689570386166;
C_vel.Kd = -0.0151687829718727;   % D
C_vel.N  = 77.0155336495376;      % N

% SAS Arfagem (amortecimento de q)
Kq = 0.1;

% --- Latero-direcional ---

% Roll (Bank Angle Hold): phi_ref -> delta_a (PID no modeloNL)
C_phi.Kp = 26.7874562402529;
C_phi.Ki = 13.1675046432808;
C_phi.Kd = -0.0875715773472267;   % D
C_phi.N  = 305.892129064194;      % N
% Saturacao: [-0.43, 0.43] rad (hardcoded no modelo)

% SAS Rolamento (amortecimento de p)
Kp = 0.119;

% Amortecedor de Guinada (washout de r)
Kr = 0.15;

% Heading -> phi: ganho = 0.8 (hardcoded no modelo Latero)
% Heading PID:    P = 2.0, I = 0.9 (hardcoded no modelo de guiagem)

%% ========== Waypoints ==========
% Formato: [Norte(m), Leste(m), Altitude(m), Velocidade(m/s)]
% WP1 deve corresponder a posicao inicial da aeronave (Xe)
%
% A guiagem inicia mirando WP2 (wp_idx = 2 no t=0).
% Altitude em NED: positiva para cima (convertida internamente).

% --- Missao padrao (retangulo) ---
WPs = [
     0,    0, 100, 15;   % WP1: partida (coincide com Xe)
   300,    0, 100, 15;   % WP2: reto Norte
   300,  200, 110, 15;   % WP3: curva Leste + subida
     0,  200, 110, 15;   % WP4: retorno Oeste
     0,    0, 100, 15;   % WP5: volta ao inicio
];

% Raio de aceitacao para troca de waypoint (m)
R_accept = 30;

% Estado inicial passado para Guidance_Star (validacao interna)
Xe_init = Xe;

%% ========== Pronto ==========
disp('--- Guiagem: workspace carregado ---');
disp(['  Waypoints: ' num2str(size(WPs,1)) ' pontos']);
disp(['  R_accept:  ' num2str(R_accept) ' m']);
disp(['  VT_eq:     ' num2str(norm(Xe(1:3))) ' m/s']);
disp(['  Alt_eq:    ' num2str(-Xe(12)) ' m']);
