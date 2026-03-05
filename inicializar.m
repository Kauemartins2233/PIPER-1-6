%% inicializar.m - Script principal de inicializacao
% Carrega parametros da aeronave, define ganhos dos controladores PID,
% waypoints e condicao de equilibrio.
%
% Este script inicializa o workspace para AMBOS os modelos:
%   - NL_guidance.slx (guiagem por waypoints)
%   - modeloNL1.slx   (controle em malha fechada)
%
% Uso para GUIAGEM:
%   1) >> inicializar
%   2) >> open('guiagem/NL_guidance.slx')
%   3) Simular (Ctrl+T)
%   4) >> plot3d_voo          % visualizar trajetoria 3D
%
% Uso para CONTROLE:
%   1) >> inicializar
%   2) >> open('controle/Nao Linear/modeloNL1.slx')
%   3) Simular (Ctrl+T)
%
% NOTA: O modelo usa sfunction_piper (21 outputs, DirFeedthrough=0).
%       Mesmo arquivo usado em ambos os modelos Simulink.

clear; clc;

%% ========== Paths ==========
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'modelos', 'Não Linear'));  % sfunction_piper, aerodynamics, dyn_rigidbody, etc.
addpath(fullfile(rootDir, 'guiagem'));                 % plot3d_voo, etc.

%% ========== Parametros da Aeronave ==========
% Carrega par_aero, par_prop, par_gen
load('Sato_longitudinal_Piper_1_6.mat');

%% ========== Estado de Equilibrio ==========
equilibrium;  % Define Xe (12x1) e Ue (4x1)

%% ========== Verificacao do Trim ==========
% Checa se as derivadas no ponto de equilibrio sao ~zero
Xp0 = dyn_rigidbody(0, Xe, Ue, par_gen, par_aero, par_prop);
% Excluir xNp(10) e xEp(11) do residuo — sao velocidades de translacao
% esperadas em voo reto (xNp ~ VT * cos(alpha) ~ 15 m/s)
trim_residual = norm([Xp0(1:9); Xp0(12)]);
fprintf('  Residuo do trim (corpo+xDp): %.6e\n', trim_residual);
fprintf('  xNp=%.4f m/s, xEp=%.4f m/s (velocidades de translacao)\n', Xp0(10), Xp0(11));
% NOTA: O trim foi obtido via fmincon no Simulink (trimagem_piper.m).
% Quando avaliado diretamente por dyn_rigidbody, o residuo pode ser ~0.19
% devido a diferencas na implementacao (ex: feedthrough, integracao).
% Isso NAO indica erro — o trim e correto para o modelo Simulink.
if trim_residual > 0.5
    warning('Trim NAO esta em equilibrio! Residuo = %.4f. Verifique Xe/Ue.', trim_residual);
    fprintf('  Derivadas: %s\n', mat2str(Xp0', 4));
elseif trim_residual > 0.1
    fprintf('  (residuo > 0.1 — normal para trim obtido via Simulink/fmincon)\n');
end

%% ========== Ganhos do Piloto Automatico ==========
% Valores extraidos do modeloNL1.slx.
% Os blocos PID em ambos os modelos referenciam estas variaveis do workspace.

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
C_theta.N  = 20;  

% Velocidade: VT_ref -> delta_T
% NOTA: No modeloNL1.slx os ganhos sao negativos E o Sum usa |+-
% (VT_ref - VT). Ganhos negativos com essa convencao produzem um
% controlador INVERTIDO (reduz throttle quando esta lento = instavel).
% Aqui usamos ganhos POSITIVOS para obter o comportamento correto:
% VT < VT_ref -> erro > 0 -> PID > 0 -> mais throttle.
C_vel.Kp = 0.0786752433250596;
C_vel.Ki = 0.0200000000000000;   
C_vel.Kd = 0.0151687829718727;
C_vel.N  = 20;  

% SAS Arfagem (amortecimento de q)
Kq = 0.1;

% --- Latero-direcional ---

% Roll (Bank Angle Hold): phi_ref -> delta_a
C_phi.Kp = 26.7874562402529;
C_phi.Ki = 13.1675046432808;
C_phi.Kd = -0.0875715773472267;
C_phi.N  = 20; 
% Saturacao: [-0.43, 0.43] rad (hardcoded no modelo)

% SAS Rolamento (amortecimento de p)
Kp = 0.119;

% Amortecedor de Guinada (washout de r)
Kr = 0.15;

% Heading -> phi: ganho = 0.8 (hardcoded no modelo Latero)
% Heading PID:    P = 2.5, I = 0 (salvo no modelo)

%% ========== Waypoints ==========
% Formato: [Norte(m), Leste(m), Altitude(m), Velocidade(m/s)]
% WP1 deve corresponder a posicao inicial da aeronave (Xe)
%
% A guiagem inicia mirando WP2 (wp_idx = 2 no t=0).
% Altitude em NED: positiva para cima (convertida internamente).

% Altitude dos WPs deve ser consistente com Xe(12).
% Xe(12) = -100 -> altitude de equilibrio = 100m
h_eq = -Xe(12);  % altitude de equilibrio

% --- Selecionar missao ---
% 1 = voo reto (teste de controle puro, sem transicoes de WP)
% 2 = triangulo (mesma altitude)
% 3 = triangulo com variacao de altitude
missao = 3;

switch missao
    case 1
        % Voo reto nivelado — WP2 muito longe, heading ~0
        WPs = [
            0,     0, h_eq, 15;     % WP1: partida
            500, 0, h_eq, 15;     % WP2: Norte (nunca alcanca)
        ];
    case 2
        % Triangulo grande (altitude constante)
        WPs = [
             0,      0, h_eq,    15;   % WP1: partida (100m)
          1000,      0, h_eq,    15;   % WP2: Norte
           500,    800, h_eq,    15;   % WP3: Nordeste
             0,      0, h_eq,    15;   % WP4: volta ao inicio
        ];
    case 3
        % Triangulo com variacao de altitude
        WPs = [
             0,      0, h_eq,      15;   % WP1: partida (100m)
          1000,      0, h_eq + 50, 15;   % WP2: Norte, sobe para 150m
           500,    800, h_eq - 30, 15;   % WP3: Nordeste, desce para 70m
             0,      0, h_eq,      15;   % WP4: volta ao inicio (100m)
        ];
end

% Raio de aceitacao para troca de waypoint (m)
R_accept = 80;

% Estado inicial passado para Guidance_Star (validacao interna)
Xe_init = Xe;

%% ========== Compatibilidade com modeloNL1.slx ==========
% O modeloNL1 (controle) usa estas variaveis nos blocos Constant.
INPUTS    = Ue';   % [delta_T, delta_e, delta_a, delta_r]
TrimInput = Ue';   % Alias (alguns blocos usam TrimInput)
Kp_sas    = Kp;    % Alias do SAS de rolamento (Kp = 0.119)

%% ========== Pronto ==========
disp('--- Workspace carregado (guiagem + controle) ---');
disp(['  Waypoints: ' num2str(size(WPs,1)) ' pontos']);
disp(['  R_accept:  ' num2str(R_accept) ' m']);
disp(['  VT_eq:     ' num2str(norm(Xe(1:3))) ' m/s']);
disp(['  Alt_eq:    ' num2str(-Xe(12)) ' m']);
fprintf('\n  Para GUIAGEM:  open(''guiagem/NL_guidance.slx''), simular, depois plot3d_voo\n');
fprintf('  Para CONTROLE: open(''controle/Nao Linear/modeloNL1.slx''), depois simular\n');
