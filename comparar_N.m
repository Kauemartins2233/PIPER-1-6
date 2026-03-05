%% comparar_N.m - Compara simulação com N original vs N ajustado (max 100)
% Roda NL_guidance.slx duas vezes e plota trajetórias 3D + atuadores.
%
% Uso: >> comparar_N

clear; clc; close all;

%% ========== Configuração comum ==========
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'modelos', 'Não Linear'));
addpath(fullfile(rootDir, 'guiagem'));

load('Sato_longitudinal_Piper_1_6.mat');
equilibrium;

Xp0 = dyn_rigidbody(0, Xe, Ue, par_gen, par_aero, par_prop);
fprintf('  Resíduo do trim: %.6e\n', norm([Xp0(1:9); Xp0(12)]));

% Ganhos base (sem N)
C_alt.Kp = 0.596304245000559;
C_alt.Ki = 0.356254495459697;
C_alt.Kd = -0.0141697733728916;

C_theta.Kp = 20.3142831421082;
C_theta.Ki = 22.5973845921282;
C_theta.Kd = 1.76724278063426;

C_vel.Kp = 0.0786752433250596;
C_vel.Ki = 0.0200000000000000;
C_vel.Kd = 0.0151687829718727;

C_phi.Kp = 26.7874562402529;
C_phi.Ki = 13.1675046432808;
C_phi.Kd = -0.0875715773472267;

Kq = 0.1;
Kp = 0.119;
Kr = 0.15;

% Waypoints - missão 3 (triângulo com variação de altitude)
h_eq = -Xe(12);
WPs = [
     0,      0, h_eq,      15;
  1000,      0, h_eq + 50, 15;
   500,    800, h_eq - 30, 15;
     0,      0, h_eq,      15;
];
R_accept = 80;
Xe_init = Xe;
INPUTS    = Ue';
TrimInput = Ue';
Kp_sas    = Kp;

modelo = fullfile(rootDir, 'guiagem', 'NL_guidance.slx');

%% ========== Simulação 1: N ORIGINAL ==========
fprintf('\n===== Simulação 1: N ORIGINAL =====\n');
C_alt.N   = 6.17157424867561;
C_theta.N = 1159.39165466191;
C_vel.N   = 77.0155336495376;
C_phi.N   = 305.892129064194;

fprintf('  C_alt.N   = %.2f\n', C_alt.N);
fprintf('  C_theta.N = %.2f\n', C_theta.N);
fprintf('  C_vel.N   = %.2f\n', C_vel.N);
fprintf('  C_phi.N   = %.2f\n', C_phi.N);

out1 = sim(modelo);
res1 = salvar_resultado(out1);
fprintf('  Simulação 1 concluída.\n');

%% ========== Simulação 2: N AJUSTADO (N=0, sem derivada) ==========
fprintf('\n===== Simulação 2: N = 0 (sem termo derivativo) =====\n');
C_alt.N   = 0;
C_theta.N = 0;
C_vel.N   = 0;
C_phi.N   = 0;

fprintf('  C_alt.N   = %.2f\n', C_alt.N);
fprintf('  C_theta.N = %.2f\n', C_theta.N);
fprintf('  C_vel.N   = %.2f\n', C_vel.N);
fprintf('  C_phi.N   = %.2f\n', C_phi.N);

out2 = sim(modelo);
res2 = salvar_resultado(out2);
fprintf('  Simulação 2 concluída.\n');

%% ========== Figura 1: Trajetória 3D sobreposta ==========
figure('Name', 'Sobreposição 3D', 'Position', [50 100 900 700]);
plot3(res1.xE, res1.xN, res1.alt, 'b-', 'LineWidth', 1.5); hold on;
plot3(res2.xE, res2.xN, res2.alt, 'r-', 'LineWidth', 1.5);
plot3(WPs(:,2), WPs(:,1), WPs(:,3), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
for i = 1:size(WPs,1)
    text(WPs(i,2)+5, WPs(i,1)+5, WPs(i,3)+2, sprintf('WP%d',i), ...
         'FontSize', 9, 'FontWeight', 'bold');
end
xlabel('Leste (m)'); ylabel('Norte (m)'); zlabel('Altitude (m)');
title('Comparação de Trajetórias 3D');
legend('N original', 'N ajustador', 'Waypoints', 'Location', 'best');
grid on; axis equal; view(30, 25); hold off;
saveas(gcf, fullfile(rootDir, 'fig_sobreposicao_3d.png'));
fprintf('  Salvo: fig_sobreposicao_3d.png\n');

%% ========== Figura 2: Sinais de controle (atuadores) ==========
figure('Name', 'Atuadores', 'Position', [50 100 1200 800]);

% delta_T (throttle)
subplot(2,2,1);
plot(res1.t_u, res1.dT, 'b-', 'LineWidth', 1); hold on;
plot(res2.t_u, res2.dT, 'r-', 'LineWidth', 1);
xlabel('Tempo (s)'); ylabel('\delta_T');
title('Throttle (\delta_T)');
legend('N orig', 'N ajust', 'Location', 'best');
grid on; hold off;

% delta_e (elevator)
subplot(2,2,2);
plot(res1.t_u, rad2deg(res1.dE), 'b-', 'LineWidth', 1); hold on;
plot(res2.t_u, rad2deg(res2.dE), 'r-', 'LineWidth', 1);
xlabel('Tempo (s)'); ylabel('\delta_e (°)');
title('Profundor (\delta_e)');
legend('N orig', 'N ajust', 'Location', 'best');
grid on; hold off;

% delta_a (aileron)
subplot(2,2,3);
plot(res1.t_u, rad2deg(res1.dA), 'b-', 'LineWidth', 1); hold on;
plot(res2.t_u, rad2deg(res2.dA), 'r-', 'LineWidth', 1);
xlabel('Tempo (s)'); ylabel('\delta_a (°)');
title('Aileron (\delta_a)');
legend('N orig', 'N ajust', 'Location', 'best');
grid on; hold off;

% delta_r (rudder)
subplot(2,2,4);
plot(res1.t_u, rad2deg(res1.dR), 'b-', 'LineWidth', 1); hold on;
plot(res2.t_u, rad2deg(res2.dR), 'r-', 'LineWidth', 1);
xlabel('Tempo (s)'); ylabel('\delta_r (°)');
title('Leme (\delta_r)');
legend('N orig', 'N ajust', 'Location', 'best');
grid on; hold off;

sgtitle('Sinais de Controle (Atuadores)');
saveas(gcf, fullfile(rootDir, 'fig_atuadores.png'));
fprintf('  Salvo: fig_atuadores.png\n');

%% ========== Figura 3: Respostas angulares ==========
figure('Name', 'Respostas', 'Position', [50 100 1200 800]);

% theta (pitch)
subplot(2,2,1);
plot(res1.t_y, rad2deg(res1.theta), 'b-', 'LineWidth', 1); hold on;
plot(res2.t_y, rad2deg(res2.theta), 'r-', 'LineWidth', 1);
xlabel('Tempo (s)'); ylabel('\theta (°)');
title('Arfagem (\theta)');
legend('N orig', 'N ajust', 'Location', 'best');
grid on; hold off;

% phi (roll)
subplot(2,2,2);
plot(res1.t_y, rad2deg(res1.phi), 'b-', 'LineWidth', 1); hold on;
plot(res2.t_y, rad2deg(res2.phi), 'r-', 'LineWidth', 1);
xlabel('Tempo (s)'); ylabel('\phi (°)');
title('Rolamento (\phi)');
legend('N orig', 'N ajust', 'Location', 'best');
grid on; hold off;

% q (pitch rate)
subplot(2,2,3);
plot(res1.t_y, rad2deg(res1.q), 'b-', 'LineWidth', 1); hold on;
plot(res2.t_y, rad2deg(res2.q), 'r-', 'LineWidth', 1);
xlabel('Tempo (s)'); ylabel('q (°/s)');
title('Taxa de arfagem (q)');
legend('N orig', 'N ajust', 'Location', 'best');
grid on; hold off;

% VT (airspeed)
subplot(2,2,4);
plot(res1.t_y, res1.VT, 'b-', 'LineWidth', 1); hold on;
plot(res2.t_y, res2.VT, 'r-', 'LineWidth', 1);
xlabel('Tempo (s)'); ylabel('V_T (m/s)');
title('Velocidade (V_T)');
legend('N orig', 'N ajust', 'Location', 'best');
grid on; hold off;

sgtitle('Respostas da Aeronave');
saveas(gcf, fullfile(rootDir, 'fig_respostas.png'));
fprintf('  Salvo: fig_respostas.png\n');

%% ========== Figura 4: Altitude vs Tempo ==========
figure('Name', 'Altitude', 'Position', [900 100 800 400]);
plot(res1.t_pos, res1.alt, 'b-', 'LineWidth', 1.5); hold on;
plot(res2.t_pos, res2.alt, 'r-', 'LineWidth', 1.5);
alt_wps = unique(WPs(:,3));
for i = 1:length(alt_wps)
    yline(alt_wps(i), 'k--', sprintf('%.0f m', alt_wps(i)), ...
           'LineWidth', 0.5, 'LabelHorizontalAlignment', 'left');
end
xlabel('Tempo (s)'); ylabel('Altitude (m)');
title('Altitude ao Longo do Voo');
legend('N original', 'N ajustador', 'Location', 'best');
grid on; hold off;
saveas(gcf, fullfile(rootDir, 'fig_altitude.png'));
fprintf('  Salvo: fig_altitude.png\n');

%% ========== Estatísticas ==========
fprintf('\n===== COMPARAÇÃO =====\n');
fprintf('  %-25s %12s %12s\n', '', 'N Original', 'N Conserv.');
fprintf('  %-25s %12.1f %12.1f\n', 'Alt min (m)',  min(res1.alt),  min(res2.alt));
fprintf('  %-25s %12.1f %12.1f\n', 'Alt max (m)',  max(res1.alt),  max(res2.alt));
fprintf('  %-25s %12.1f %12.1f\n', 'Alt média (m)', mean(res1.alt), mean(res2.alt));

% Atividade dos atuadores (integral do valor absoluto da derivada)
act_dE1 = sum(abs(diff(res1.dE))) / length(res1.dE);
act_dE2 = sum(abs(diff(res2.dE))) / length(res2.dE);
act_dA1 = sum(abs(diff(res1.dA))) / length(res1.dA);
act_dA2 = sum(abs(diff(res2.dA))) / length(res2.dA);
fprintf('  %-25s %12.6f %12.6f\n', 'Atividade delta_e (rad)', act_dE1, act_dE2);
fprintf('  %-25s %12.6f %12.6f\n', 'Atividade delta_a (rad)', act_dA1, act_dA2);

% Desvio padrão dos sinais de controle (indica ruído/oscilação)
fprintf('  %-25s %12.4f %12.4f\n', 'Std delta_e (°)', rad2deg(std(res1.dE)), rad2deg(std(res2.dE)));
fprintf('  %-25s %12.4f %12.4f\n', 'Std delta_a (°)', rad2deg(std(res1.dA)), rad2deg(std(res2.dA)));
fprintf('  %-25s %12.4f %12.4f\n', 'Std q (°/s)', rad2deg(std(res1.q)), rad2deg(std(res2.q)));
fprintf('\n');

%% ========== Função auxiliar ==========
function res = salvar_resultado(out)
    % Tentar acessar U, Y, POS do objeto SimulationOutput
    % (To Workspace blocks com ReturnWorkspaceOutputs=on)
    U = out.get('U');
    Y = out.get('Y');
    POS = out.get('POS');

    % U: Structure With Time -> .time, .signals.values(:, [dT, dE, dA, dR])
    res.t_u = U.time;
    res.dT  = U.signals.values(:, 1);
    res.dE  = U.signals.values(:, 2);
    res.dA  = U.signals.values(:, 3);
    res.dR  = U.signals.values(:, 4);

    % Y: Structure With Time -> .signals.values com 21 outputs da s-function
    % sys(1)=VT, sys(4)=p, sys(5)=q, sys(6)=r, sys(7)=phi, sys(8)=theta
    res.t_y   = Y.time;
    res.VT    = Y.signals.values(:, 1);
    res.alpha = Y.signals.values(:, 2);
    res.beta  = Y.signals.values(:, 3);
    res.p     = Y.signals.values(:, 4);
    res.q     = Y.signals.values(:, 5);
    res.r     = Y.signals.values(:, 6);
    res.phi   = Y.signals.values(:, 7);
    res.theta = Y.signals.values(:, 8);
    res.psi   = Y.signals.values(:, 9);

    % POS: Timeseries [xN, xE, alt]
    if ~isempty(POS) && isa(POS, 'timeseries')
        res.t_pos = POS.Time;
        res.xN    = POS.Data(:, 1);
        res.xE    = POS.Data(:, 2);
        res.alt   = POS.Data(:, 3);
    else
        res.t_pos = res.t_y;
        res.xN    = Y.signals.values(:, 10);
        res.xE    = Y.signals.values(:, 11);
        res.alt   = -Y.signals.values(:, 12);
    end
end
