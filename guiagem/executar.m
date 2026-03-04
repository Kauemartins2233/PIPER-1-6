%% executar.m - Inicializacao do modelo de guiagem NL_guidance.slx
% Carrega parametros da aeronave, define ganhos dos controladores PID,
% waypoints e condicao de equilibrio.
%
% Uso:
%   1) Rodar este script:    >> executar
%   2) Abrir o modelo:       >> open('NL_guidance.slx')
%   3) Aplicar PID completo: >> setup_pid_blocks
%      (restaura termos D e N nos blocos PID do Simulink)
%   4) Simular (Ctrl+T ou botao Run)
%
% NOTA: O modelo usa sfunction_piper_trim com 21 outputs.
%       Certifique-se de que trim/ esta no path.

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

%% ========== Verificacao do Trim ==========
% Checa se as derivadas no ponto de equilibrio sao ~zero
Xp0 = dyn_rigidbody(0, Xe, Ue, par_gen, par_aero, par_prop);
% Excluir xNp(10) e xEp(11) do residuo — sao velocidades de translacao
% esperadas em voo reto (xNp ~ VT * cos(alpha) ~ 15 m/s)
trim_residual = norm([Xp0(1:9); Xp0(12)]);
fprintf('  Residuo do trim (corpo+xDp): %.6e\n', trim_residual);
fprintf('  xNp=%.4f m/s, xEp=%.4f m/s (velocidades de translacao)\n', Xp0(10), Xp0(11));
if trim_residual > 0.1
    warning('Trim NAO esta em equilibrio! Residuo = %.4f. Verifique Xe/Ue.', trim_residual);
    fprintf('  Derivadas: %s\n', mat2str(Xp0', 4));
end

%% ========== Ganhos do Piloto Automatico ==========
% Valores extraidos do modeloNL1.slx (hardcoded nos blocos PID originais).
% IMPORTANTE: Os blocos PID no NL_guidance.slx usam apenas Kp e Ki do
% workspace (D=0, N=100 hardcoded). Para usar PID COMPLETO com os termos
% D e N corretos, rodar setup_pid_blocks.m APOS abrir o modelo.

% --- Longitudinal ---

% Altitude Hold: h_ref -> theta_ref
% Saturacao: [-0.17, 0.26] rad (hardcoded no modelo)
C_alt.Kp = 0.596304245000559;
C_alt.Ki = 0.356254495459697;
C_alt.Kd = -0.0141697733728916;
C_alt.N  = 6.17157424867561;

% Pitch (Atitude): theta_ref -> delta_e
C_theta.Kp = 20.3142831421082;
C_theta.Ki = 22.5973845921282;
C_theta.Kd = 1.76724278063426;
C_theta.N  = 1159.39165466191;

% Velocidade: VT_ref -> delta_T
% NOTA: No modeloNL1.slx os ganhos sao negativos E o Sum usa |+-
% (VT_ref - VT). Ganhos negativos com essa convencao produzem um
% controlador INVERTIDO (reduz throttle quando esta lento = instavel).
% Aqui usamos ganhos POSITIVOS para obter o comportamento correto:
% VT < VT_ref -> erro > 0 -> PID > 0 -> mais throttle.
C_vel.Kp = 0.0786752433250596;
C_vel.Ki = 0.0200000000000000;   % Reduzido (era 0.0652) - evita acumulo de throttle
C_vel.Kd = 0.0151687829718727;
C_vel.N  = 77.0155336495376;

% SAS Arfagem (amortecimento de q)
Kq = 0.1;

% --- Latero-direcional ---

% Roll (Bank Angle Hold): phi_ref -> delta_a
C_phi.Kp = 26.7874562402529;
C_phi.Ki = 13.1675046432808;
C_phi.Kd = -0.0875715773472267;
C_phi.N  = 305.892129064194;
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

% --- Missao (triangulo grande) ---
% Altitude dos WPs deve ser consistente com Xe(12).
% Xe(12) = -560 -> altitude de equilibrio = 560m
h_eq = -Xe(12);  % altitude de equilibrio

WPs = [
     0,      0, h_eq,   15;   % WP1: partida
  1000,      0, h_eq,   15;   % WP2: Norte longe
   500,    800, h_eq,   15;   % WP3: Nordeste
     0,      0, h_eq,   15;   % WP4: volta ao inicio
];

% Raio de aceitacao para troca de waypoint (m)
% Constant4 no Simulink deve usar a variavel R_accept (nao hardcoded).
% Para alterar: set_param('NL_guidance/Autopilot1/Constant4', 'Value', 'R_accept')
R_accept = 80;

% Estado inicial passado para Guidance_Star (validacao interna)
Xe_init = Xe;

%% ========== Pronto ==========
disp('--- Guiagem: workspace carregado ---');
disp(['  Waypoints: ' num2str(size(WPs,1)) ' pontos']);
disp(['  R_accept:  ' num2str(R_accept) ' m']);
disp(['  VT_eq:     ' num2str(norm(Xe(1:3))) ' m/s']);
disp(['  Alt_eq:    ' num2str(-Xe(12)) ' m']);
fprintf('\n  >>> PROXIMO PASSO: abra NL_guidance.slx e rode setup_pid_blocks <<<\n');
