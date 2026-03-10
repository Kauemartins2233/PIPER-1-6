%% comparar_guiagem.m - Compara guiagem NL_guidance vs X-Plane
% Simula NL_guidance.slx offline e compara com dados do X-Plane.
%
% Pre-requisito:
%   1) inicializar  (WPs, R_accept, ganhos PID)
%   2) teste_guiagem ja rodou (dados log_* no workspace)
%
% Uso:
%   >> comparar_guiagem

%% ========== Verificar workspace ==========
if ~exist('Xe', 'var') || ~exist('WPs', 'var')
    inicializar;
end

% Verificar se dados do X-Plane existem
tem_xplane = exist('log_xN', 'var') && exist('log_xE', 'var');
if ~tem_xplane
    fprintf('AVISO: Dados do X-Plane nao encontrados. Rode teste_guiagem antes.\n');
    fprintf('       Comparacao sera apenas NL_guidance.\n');
end

rootDir = fileparts(mfilename('fullpath'));

fprintf('\n=== COMPARACAO DE GUIAGEM ===\n');
fprintf('  Waypoints: %d pontos\n', size(WPs, 1));
for i = 1:size(WPs, 1)
    fprintf('  WP%d: N=%+.0f E=%+.0f h=%.0f m\n', ...
        i, WPs(i,1), WPs(i,2), WPs(i,3));
end

%% ========== 1) Simular NL_guidance ==========
fprintf('\n[1/2] Simulando NL_guidance...\n');
modelGuid = fullfile(rootDir, 'guiagem', 'NL_guidance.slx');
load_system(modelGuid);

% StopTime longo o suficiente para completar a missao
set_param('NL_guidance', 'StopTime', '300');
out_nl = sim('NL_guidance');

% Extrair dados
Y_nl = out_nl.get('Y');
if ~isempty(Y_nl) && isstruct(Y_nl) && isfield(Y_nl, 'signals')
    t_nl   = Y_nl.time;
    vals   = Y_nl.signals.values;
else
    t_nl   = out_nl.tout;
    vals   = out_nl.yout.signals.values;
end

VT_nl    = vals(:, 1);
phi_nl   = vals(:, 7);
theta_nl = vals(:, 8);
psi_nl   = vals(:, 9);
xN_nl    = vals(:, 10);
xE_nl    = vals(:, 11);
alt_nl   = vals(:, 12);  % y(12) = -xD (altitude positiva)

close_system('NL_guidance', 0);
fprintf('  OK. Duracao: %.1fs, dist total: %.0fm\n', t_nl(end), ...
    sum(sqrt(diff(xN_nl).^2 + diff(xE_nl).^2)));

%% ========== 2) Dados X-Plane ==========
if tem_xplane
    fprintf('\n[2/2] Usando dados do X-Plane (log_* do workspace)...\n');
    t_xp   = log_t;
    xN_xp  = log_xN;
    xE_xp  = log_xE;
    alt_xp = log_h;
    VT_xp  = log_VT;
    psi_xp = log_psi;
    theta_xp = log_theta;
    fprintf('  OK. Duracao: %.1fs, dist total: %.0fm\n', t_xp(end), ...
        sum(sqrt(diff(xN_xp).^2 + diff(xE_xp).^2)));
else
    fprintf('\n[2/2] X-Plane: PULADO (sem dados)\n');
end

%% ========== Plotar ==========
fprintf('\nGerando graficos...\n');

cores = struct('nl', [0.0 0.45 0.74], ...    % azul
               'xp', [0.47 0.67 0.19]);       % verde

figure('Name', 'Comparacao de Guiagem - NL vs X-Plane', ...
       'Position', [50 50 1400 800]);

% --- Trajetoria 3D ---
subplot(2, 3, [1 4]);
plot3(xE_nl, xN_nl, alt_nl, '-', 'Color', cores.nl, 'LineWidth', 2);
hold on;
if tem_xplane
    plot3(xE_xp, xN_xp, alt_xp, '-.', 'Color', cores.xp, 'LineWidth', 2);
end

% Waypoints
plot3(WPs(:,2), WPs(:,1), WPs(:,3), 'rs', 'MarkerSize', 12, ...
      'MarkerFaceColor', 'r');
for i = 1:size(WPs, 1)
    text(WPs(i,2)+10, WPs(i,1)+10, WPs(i,3)+3, sprintf('WP%d', i), ...
         'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
end

% Circulos de aceitacao (no plano do WP)
th = linspace(0, 2*pi, 100);
for i = 1:size(WPs, 1)
    plot3(WPs(i,2) + R_accept*cos(th), WPs(i,1) + R_accept*sin(th), ...
          ones(size(th))*WPs(i,3), 'r--', 'LineWidth', 0.5);
end

% Inicio
plot3(0, 0, WPs(1,3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
hold off;
xlabel('Leste (m)'); ylabel('Norte (m)'); zlabel('Altitude (m)');
title('Trajetoria 3D - Guiagem por Waypoints');
if tem_xplane
    legend('Nao-Linear', 'X-Plane', 'Waypoints', 'Location', 'best');
else
    legend('Nao-Linear', 'Waypoints', 'Location', 'best');
end
grid on; axis equal; view(30, 25);

% --- Vista Superior (Ground Track) ---
subplot(2, 3, 2);
plot(xE_nl, xN_nl, '-', 'Color', cores.nl, 'LineWidth', 1.5); hold on;
if tem_xplane
    plot(xE_xp, xN_xp, '-.', 'Color', cores.xp, 'LineWidth', 1.5);
end
plot(WPs(:,2), WPs(:,1), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
for i = 1:size(WPs, 1)
    plot(WPs(i,2) + R_accept*cos(th), WPs(i,1) + R_accept*sin(th), ...
         'r--', 'LineWidth', 0.5);
end
hold off;
xlabel('Leste (m)'); ylabel('Norte (m)');
title('Vista Superior'); grid on; axis equal;

% --- Altitude ---
subplot(2, 3, 3);
plot(t_nl, alt_nl, '-', 'Color', cores.nl, 'LineWidth', 1.5); hold on;
if tem_xplane
    plot(t_xp, alt_xp, '-.', 'Color', cores.xp, 'LineWidth', 1.5);
end
% Refs dos WPs
alt_wps = unique(WPs(:,3));
for i = 1:length(alt_wps)
    yline(alt_wps(i), 'r:', sprintf('%.0fm', alt_wps(i)));
end
hold off;
xlabel('Tempo (s)'); ylabel('Altitude (m)');
title('Altitude'); grid on;

% --- Velocidade ---
subplot(2, 3, 5);
plot(t_nl, VT_nl, '-', 'Color', cores.nl, 'LineWidth', 1.5); hold on;
if tem_xplane
    plot(t_xp, VT_xp, '-.', 'Color', cores.xp, 'LineWidth', 1.5);
end
yline(15, 'k:', 'VT_{ref}');
hold off;
xlabel('Tempo (s)'); ylabel('VT (m/s)');
title('Velocidade'); grid on;

% --- Heading ---
subplot(2, 3, 6);
% Normalizar heading para [-180, 180] deg
psi_nl_wrap = rad2deg(atan2(sin(psi_nl), cos(psi_nl)));
plot(t_nl, psi_nl_wrap, '-', 'Color', cores.nl, 'LineWidth', 1.5); hold on;
if tem_xplane
    psi_xp_wrap = rad2deg(atan2(sin(psi_xp), cos(psi_xp)));
    plot(t_xp, psi_xp_wrap, '-.', 'Color', cores.xp, 'LineWidth', 1.5);
end
hold off;
xlabel('Tempo (s)'); ylabel('\psi (deg)');
title('Heading'); grid on;

fprintf('\n=== COMPARACAO CONCLUIDA ===\n\n');
