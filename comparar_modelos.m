%% comparar_modelos.m - Comparacao 3D dos 3 modelos do Piper 1/6
% Simula voo nivelado por 30 segundos e compara trajetorias.
%
% Modelos:
%   1) Nao-Linear (modeloNL1.slx) - S-function 6DOF
%   2) Linear (modelo_linear.slx) - State-space linearizado
%   3) X-Plane 9 (via XPlaneConnect) - Simulador real [OPCIONAL]
%
% Uso:
%   >> inicializar          % (automatico se nao estiver carregado)
%   >> comparar_modelos
%
% Para incluir X-Plane: abrir o simulador antes de rodar.

%% ========== Inicializar ==========
% inicializar.m faz clear/clc, entao deve rodar ANTES das variaveis locais
if ~exist('Xe', 'var') || ~exist('C_alt', 'var')
    inicializar;
end

%% ========== Configuracao ==========
duracao = 30;       % segundos
incluir_xplane = true;  % true = rodar autopiloto no X-Plane tambem

rootDir = fileparts(mfilename('fullpath'));

% Configurar X-Plane se necessario
if incluir_xplane
    if ~exist('Ts_xplane', 'var')
        Ts_xplane = 0.05;  % 20 Hz
    end
    h_ref   = 110;      % mesma ref dos modelos Simulink (Constant1=110)
    VT_ref  = Xe(1);    % velocidade de equilibrio (15 m/s)
    psi_ref = 0;
    addpath(fullfile(rootDir, 'Xplane', 'XPlaneConnect-master', 'MATLAB'));
    addpath(fullfile(rootDir, 'Xplane'));
end
fprintf('\n=== COMPARACAO DE MODELOS ===\n');
fprintf('  Duracao: %d s\n', duracao);
fprintf('  Refs: h=110 m, VT=%.1f m/s, psi=step(0->20 deg, t=5s)\n', Xe(1));

%% ========== 1) Modelo Nao-Linear ==========
fprintf('\n[1/3] Simulando modelo Nao-Linear...\n');
modelNL = fullfile(rootDir, 'controle', 'Não Linear', 'modeloNL1.slx');
load_system(modelNL);
set_param('modeloNL1', 'StopTime', num2str(duracao));
out_nl = sim('modeloNL1');

% Extrair dados (21 outputs da s-function)
Y_nl = out_nl.get('Y');
if ~isempty(Y_nl) && isstruct(Y_nl) && isfield(Y_nl, 'signals')
    t_nl   = Y_nl.time;
    VT_nl  = Y_nl.signals.values(:, 1);
    theta_nl = Y_nl.signals.values(:, 8);
    phi_nl = Y_nl.signals.values(:, 7);
    psi_nl = Y_nl.signals.values(:, 9);
    xN_nl  = Y_nl.signals.values(:, 10);
    xE_nl  = Y_nl.signals.values(:, 11);
    alt_nl = Y_nl.signals.values(:, 12);  % y(12) = -xD (altitude positiva)
else
    % Fallback: tentar yout
    t_nl   = out_nl.tout;
    ydata  = out_nl.yout.signals.values;
    VT_nl  = ydata(:, 1);
    theta_nl = ydata(:, 8);
    phi_nl = ydata(:, 7);
    psi_nl = ydata(:, 9);
    xN_nl  = ydata(:, 10);
    xE_nl  = ydata(:, 11);
    alt_nl = ydata(:, 12);  % y(12) = -xD (altitude positiva)
end
close_system('modeloNL1', 0);
fprintf('  OK. Final: VT=%.1f m/s, h=%.1f m, xN=%.1f m, xE=%.1f m\n', VT_nl(end), alt_nl(end), xN_nl(end), xE_nl(end));

%% ========== 2) Modelo Linear ==========
fprintf('\n[2/3] Simulando modelo Linear...\n');
modelLin = fullfile(rootDir, 'controle', 'Linear', 'modelo_linear.slx');
load_system(modelLin);
set_param('modelo_linear', 'StopTime', num2str(duracao));
out_lin = sim('modelo_linear');

% Tentar extrair dados - modelo linear pode ter diferentes formatos
[t_lin, VT_lin, theta_lin, phi_lin, psi_lin, h_lin] = extrair_linear(out_lin);

% Reconstruir posicao por integracao (modelo linear nao tem xN, xE)
xN_dot_lin = VT_lin .* cos(theta_lin) .* cos(psi_lin);
xE_dot_lin = VT_lin .* cos(theta_lin) .* sin(psi_lin);
xN_lin = cumtrapz(t_lin, xN_dot_lin);
xE_lin = cumtrapz(t_lin, xE_dot_lin);
alt_lin = h_lin;

close_system('modelo_linear', 0);
fprintf('  OK. Final: VT=%.1f m/s, h=%.1f m, xN=%.1f m, xE=%.1f m\n', VT_lin(end), alt_lin(end), xN_lin(end), xE_lin(end));

%% ========== 3) X-Plane (opcional) ==========
if incluir_xplane
    fprintf('\n[3/3] Rodando autopiloto no X-Plane (%d s)...\n', duracao);
    xp_params = struct('Ts', Ts_xplane, 'h_ref', h_ref, 'VT_ref', VT_ref, ...
        'psi_ref', psi_ref, 'C_alt', C_alt, 'C_theta', C_theta, ...
        'C_vel', C_vel, 'C_phi', C_phi, 'Kq', Kq, 'Kp_sas', Kp_sas, ...
        'Kr', Kr, 'Ue', Ue);
    clear read_xplane;  % limpar persistent vars ANTES de rodar
    [t_xp, VT_xp, theta_xp, phi_xp, psi_xp, xN_xp, xE_xp, alt_xp] = ...
        rodar_xplane(duracao, xp_params);
    % Wrapping psi: X-Plane retorna [0, 2pi], modelos usam [-pi, pi]
    psi_xp = atan2(sin(psi_xp), cos(psi_xp));
    fprintf('  OK. Final: VT=%.1f m/s, h=%.1f m, xN=%.1f m, xE=%.1f m\n', VT_xp(end), alt_xp(end), xN_xp(end), xE_xp(end));
    fprintf('  Psi final: %.1f deg\n', rad2deg(psi_xp(end)));
    % Debug: direções em t=10s
    idx10 = find(t_xp >= 10, 1);
    if ~isempty(idx10)
        fprintf('  [t=10s] xN=%.1f, xE=%.1f, psi=%.1f deg\n', xN_xp(idx10), xE_xp(idx10), rad2deg(psi_xp(idx10)));
    end
    tem_xplane = true;
else
    fprintf('\n[3/3] X-Plane: PULADO (incluir_xplane = false)\n');
    tem_xplane = false;
end

%% ========== Plotar Comparacao ==========
fprintf('\nGerando graficos...\n');

cores = struct('nl', [0.0 0.45 0.74], ...    % azul
               'lin', [0.85 0.33 0.10], ...   % laranja
               'xp', [0.47 0.67 0.19]);       % verde

figure('Name', 'Comparacao de Modelos - Piper 1/6', ...
       'Position', [100 100 1200 800]);

% --- Trajetoria 3D ---
subplot(2, 3, [1 4]);
plot3(xE_nl, xN_nl, alt_nl, '-', 'Color', cores.nl, 'LineWidth', 2);
hold on;
plot3(xE_lin, xN_lin, alt_lin, '--', 'Color', cores.lin, 'LineWidth', 2);
if tem_xplane
    plot3(xE_xp, xN_xp, alt_xp, '-.', 'Color', cores.xp, 'LineWidth', 2);
end
% Ponto inicial
plot3(0, 0, -Xe(12), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
hold off;
xlabel('Leste (m)'); ylabel('Norte (m)'); zlabel('Altitude (m)');
title('Trajetoria 3D');
if tem_xplane
    legend('Nao-Linear', 'Linear', 'X-Plane', 'Inicio', 'Location', 'best');
else
    legend('Nao-Linear', 'Linear', 'Inicio', 'Location', 'best');
end
grid on; view(30, 25);

% --- Altitude ---
subplot(2, 3, 2);
plot(t_nl, alt_nl, '-', 'Color', cores.nl, 'LineWidth', 1.5); hold on;
plot(t_lin, alt_lin, '--', 'Color', cores.lin, 'LineWidth', 1.5);
if tem_xplane
    plot(t_xp, alt_xp, '-.', 'Color', cores.xp, 'LineWidth', 1.5);
end
yline(110, 'k:', 'h_{ref}');
hold off;
xlabel('Tempo (s)'); ylabel('Altitude (m)');
title('Altitude'); grid on;

% --- Velocidade ---
subplot(2, 3, 3);
plot(t_nl, VT_nl, '-', 'Color', cores.nl, 'LineWidth', 1.5); hold on;
plot(t_lin, VT_lin, '--', 'Color', cores.lin, 'LineWidth', 1.5);
if tem_xplane
    plot(t_xp, VT_xp, '-.', 'Color', cores.xp, 'LineWidth', 1.5);
end
yline(Xe(1), 'k:', 'VT_{ref}');
hold off;
xlabel('Tempo (s)'); ylabel('VT (m/s)');
title('Velocidade'); grid on;

% --- Heading ---
subplot(2, 3, 5);
plot(t_nl, rad2deg(psi_nl), '-', 'Color', cores.nl, 'LineWidth', 1.5); hold on;
plot(t_lin, rad2deg(psi_lin), '--', 'Color', cores.lin, 'LineWidth', 1.5);
if tem_xplane
    plot(t_xp, rad2deg(psi_xp), '-.', 'Color', cores.xp, 'LineWidth', 1.5);
end
% Referencia de heading: step 0->20deg em t=5s
psi_ref_plot = [0, 0, 20, 20];
t_ref_plot   = [0, 5, 5, duracao];
plot(t_ref_plot, psi_ref_plot, 'k:', 'LineWidth', 1);
hold off;
xlabel('Tempo (s)'); ylabel('\psi (deg)');
title('Heading'); grid on;

% --- Pitch ---
subplot(2, 3, 6);
plot(t_nl, rad2deg(theta_nl), '-', 'Color', cores.nl, 'LineWidth', 1.5); hold on;
plot(t_lin, rad2deg(theta_lin), '--', 'Color', cores.lin, 'LineWidth', 1.5);
if tem_xplane
    plot(t_xp, rad2deg(theta_xp), '-.', 'Color', cores.xp, 'LineWidth', 1.5);
end
hold off;
xlabel('Tempo (s)'); ylabel('\theta (deg)');
title('Arfagem'); grid on;

% Debug: resumo de direcoes
fprintf('\n--- Resumo de Posicoes Finais ---\n');
fprintf('  NL:     xN=%+.1f m, xE=%+.1f m, dir=%.1f°\n', xN_nl(end), xE_nl(end), rad2deg(atan2(xE_nl(end), xN_nl(end))));
fprintf('  Linear: xN=%+.1f m, xE=%+.1f m, dir=%.1f°\n', xN_lin(end), xE_lin(end), rad2deg(atan2(xE_lin(end), xN_lin(end))));
if tem_xplane
    fprintf('  XPlane: xN=%+.1f m, xE=%+.1f m, dir=%.1f°\n', xN_xp(end), xE_xp(end), rad2deg(atan2(xE_xp(end), xN_xp(end))));
end

fprintf('\n=== COMPARACAO CONCLUIDA ===\n\n');

%% ========== Funcoes Auxiliares ==========

function [t, VT, theta, phi, psi, h] = extrair_linear(out)
%EXTRAIR_LINEAR Extrai dados do modelo linear via logsout.
%   Sinais logados: Long_states [u,w,q,theta,h], Lat_states [V,p,r,phi,psi]
%   Sao perturbacoes em torno do equilibrio.

    % Equilibrio das SAIDAS (y = C*x)
    % Long: [u, w, q, theta, h] = [15, -1.8343, 0, -0.1217, 100]
    Xe_long = [15, -1.8343, 0, -0.1217, 100];

    logs = out.logsout;

    % Extrair Long_states e Lat_states
    long_sig = logs.getElement('Long_states');
    lat_sig  = logs.getElement('Lat_states');

    t = long_sig.Values.Time;
    vl = long_sig.Values.Data;  % [u, w, q, theta, h] perturbacoes
    vt = lat_sig.Values.Data;   % [V, p, r, phi, psi] perturbacoes

    VT    = Xe_long(1) + vl(:, 1);   % u_eq + delta_u
    theta = Xe_long(4) + vl(:, 4);   % theta_eq + delta_theta
    h     = Xe_long(5) + vl(:, 5);   % h_eq + delta_h
    phi   = vt(:, 4);                 % phi_eq = 0
    psi   = vt(:, 5);                 % psi_eq = 0
end


function [t, VT, theta, phi, psi, xN, xE, alt] = rodar_xplane(duracao, P)
%RODAR_XPLANE Roda o autopiloto no X-Plane por 'duracao' segundos.
%   P: struct com campos Ts, h_ref, VT_ref, psi_ref, C_alt, C_theta,
%      C_vel, C_phi, Kq, Kp_sas, Kr, Ue

    global GlobalSocket;
    import XPlaneConnect.*;

    if isempty(GlobalSocket)
        GlobalSocket = openUDP('127.0.0.1', 49009);
    end

    % Posicionar aeronave em voo
    drefs_ll = {'sim/flightmodel/position/latitude', ...
                'sim/flightmodel/position/longitude'};
    ll = double(getDREFs(drefs_ll, GlobalSocket));

    pauseSim(1, GlobalSocket);
    pause(0.2);

    hdg = 0;
    sendPOSI([ll(1), ll(2), 100, -7, 0, hdg, 1], 0, GlobalSocket);

    VT0 = 15; hdg_rad = hdg * pi/180;
    sendDREF('sim/flightmodel/position/local_vx', VT0*sin(hdg_rad), GlobalSocket);
    sendDREF('sim/flightmodel/position/local_vy', 0, GlobalSocket);
    sendDREF('sim/flightmodel/position/local_vz', -VT0*cos(hdg_rad), GlobalSocket);
    sendCTRL([0, 0, 0, 0.49, -998, -998], 0, GlobalSocket);

    pause(0.2);
    pauseSim(0, GlobalSocket);

    Ts = P.Ts;
    N = round(duracao / Ts);

    % PIDs
    pid_alt=[0,0,0]; pid_theta=[0,0,0]; pid_vel=[0,0,0]; pid_phi=[0,0,0];
    r_prev = 0;

    % Logs
    t   = zeros(N,1); VT  = zeros(N,1); alt = zeros(N,1);
    theta = zeros(N,1); phi = zeros(N,1); psi = zeros(N,1);
    xN  = zeros(N,1); xE  = zeros(N,1);

    for k = 1:N
        tic;
        s = read_xplane(0);
        VT_m=s(1); theta_m=s(2); q_m=s(3); h_m=s(4);
        phi_m=s(5); p_m=s(6); psi_m=s(7); r_m=s(8);

        % Debug primeiras leituras
        if k <= 3
            fprintf('  [k=%d] VT=%.1f psi=%.1f° xN=%.1f xE=%.1f h=%.1f\n', ...
                k, VT_m, rad2deg(psi_m), s(9), s(10), h_m);
        end

        % Longitudinal
        e_alt = P.h_ref - h_m;
        [theta_cmd, pid_alt] = pid_step(e_alt, P.C_alt, Ts, pid_alt, [-0.17, 0.26]);
        e_theta = theta_cmd - theta_m;
        [de_pid, pid_theta] = pid_step(e_theta, P.C_theta, Ts, pid_theta, [-0.4363, 0.4363]);
        delta_e = max(-0.4363, min(0.4363, de_pid - P.Kq*q_m));

        e_vel = P.VT_ref - VT_m;
        [dt_pid, pid_vel] = pid_step(e_vel, P.C_vel, Ts, pid_vel, [-0.51, 0.51]);
        delta_T = max(0, min(1, dt_pid + P.Ue(1)));

        % Lateral (heading ref = step 0->20deg em t=5s, igual ao Simulink)
        t_now = (k-1)*Ts;
        if t_now < 5
            psi_ref_k = 0;
        else
            psi_ref_k = deg2rad(20);
        end
        e_psi = atan2(sin(psi_ref_k - psi_m), cos(psi_ref_k - psi_m));
        phi_cmd = 0.3 * e_psi;
        e_phi = phi_cmd - phi_m;
        [da_pid, pid_phi] = pid_step(e_phi, P.C_phi, Ts, pid_phi, [-0.4363, 0.4363]);
        delta_a = max(-0.4363, min(0.4363, da_pid - P.Kp_sas*p_m));

        r_washed = r_m - r_prev * exp(-Ts);
        r_prev = r_m;
        delta_r = max(-0.4363, min(0.4363, P.Kr * r_washed));

        send_xplane([delta_e, delta_a, delta_r, delta_T]);

        % Log
        t(k)     = (k-1)*Ts;
        VT(k)    = VT_m;
        alt(k)   = h_m;
        theta(k) = theta_m;
        phi(k)   = phi_m;
        psi(k)   = psi_m;
        xN(k)    = s(9);
        xE(k)    = s(10);

        elapsed = toc;
        if elapsed < Ts, pause(Ts - elapsed); end
    end

    try close_xplane; catch; end
end


function [u, state] = pid_step(e, C, Ts, state, lims)
    integ   = state(1);
    e_prev  = state(2);
    df_prev = state(3);

    integ = integ + C.Ki * Ts * (e + e_prev) / 2;

    if C.Kd ~= 0
        alpha = C.N * Ts / (1 + C.N * Ts);
        df = alpha * C.Kd * (e - e_prev) / Ts + (1 - alpha) * df_prev;
    else
        df = 0;
    end

    u = C.Kp * e + integ + df;

    if u > lims(2)
        u = lims(2);
        if e > 0, integ = state(1); end
    elseif u < lims(1)
        u = lims(1);
        if e < 0, integ = state(1); end
    end

    state = [integ, e, df];
end
