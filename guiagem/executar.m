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

%% ========== Ganhos do Piloto Automatico (PI) ==========
% Os controladores no modelo de guiagem usam PI (D=0, N=100).
% Valores baseados no projeto via PID Tuner, simplificados para PI.

% --- Longitudinal ---

% Altitude Hold: h_ref -> theta_ref
C_alt.Kp = 0.596;
C_alt.Ki = 0.356;
% Saturacao: [-0.17, 0.26] rad (hardcoded no modelo)

% Pitch (Atitude): theta_ref -> delta_e
C_theta.Kp = 20.31;
C_theta.Ki = 22.60;

% Velocidade: VT_ref -> delta_T
C_vel.Kp = -0.528;
C_vel.Ki = -0.431;

% SAS Arfagem (amortecimento de q)
Kq = 0.1;

% --- Latero-direcional ---

% Roll (Bank Angle Hold): phi_ref -> delta_a
C_phi.Kp = 26.79;
C_phi.Ki = 13.17;
% Saturacao: [-0.43, 0.43] rad (hardcoded no modelo)

% SAS Rolamento (amortecimento de p)
Kp = 0.119;

% Amortecedor de Guinada (washout de r)
Kr = 0.1;

% Heading -> phi: ganho = 0.15 (hardcoded no modelo)
% Heading PID:    P = 2.0, I = 0.9 (hardcoded no modelo)

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
