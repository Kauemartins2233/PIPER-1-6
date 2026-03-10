%% teste_guiagem.m - Guiagem por waypoints no X-Plane
% Roda o autopiloto com navegacao LOS (mesma logica do NL_guidance.slx).
%
% Pre-requisito: X-Plane aberto com Piper J-3 Cub em voo.
%
% Uso:
%   >> inicializar          % carrega WPs, R_accept, ganhos PID
%   >> inicializar_xplane   % configura conexao X-Plane
%   >> teste_guiagem        % roda guiagem no X-Plane
%   >> plot3d_voo           % visualiza trajetoria 3D

%% ========== Verificar workspace ==========
if ~exist('Xe', 'var') || ~exist('WPs', 'var')
    error('Rode "inicializar" antes de teste_guiagem.');
end
if ~exist('Ts_xplane', 'var')
    error('Rode "inicializar_xplane" antes de teste_guiagem.');
end

rootDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(rootDir);
addpath(fullfile(rootDir, 'XPlaneConnect-master', 'MATLAB'));
addpath(rootDir);
addpath(fullfile(projectDir, 'guiagem'));

import XPlaneConnect.*;

global GlobalSocket;
if isempty(GlobalSocket)
    GlobalSocket = openUDP('127.0.0.1', 49009);
end

Ts = Ts_xplane;  % de inicializar_xplane (0.05s = 20 Hz)

fprintf('\n=== GUIAGEM X-PLANE ===\n');
fprintf('  Waypoints: %d pontos\n', size(WPs, 1));
fprintf('  R_accept:  %d m\n', R_accept);
for i = 1:size(WPs, 1)
    fprintf('  WP%d: N=%+.0f E=%+.0f h=%.0f m VT=%.0f m/s\n', ...
        i, WPs(i,1), WPs(i,2), WPs(i,3), WPs(i,4));
end

%% ========== Posicionar aeronave (identico ao teste_autopiloto.m) ==========
drefs_ll = {'sim/flightmodel/position/latitude', ...
            'sim/flightmodel/position/longitude'};
ll = double(getDREFs(drefs_ll, GlobalSocket));

% Heading inicial: apontar para WP2
hdg_rad_init = atan2(WPs(2,2) - WPs(1,2), WPs(2,1) - WPs(1,1));
hdg_deg_init = rad2deg(hdg_rad_init);
if hdg_deg_init < 0, hdg_deg_init = hdg_deg_init + 360; end

h0 = WPs(1, 3);  % altitude do WP1
VT0 = WPs(1, 4);  % velocidade do WP1

pauseSim(1, GlobalSocket);
pause(0.3);

% Posicionar: enviar 2x para garantir (X-Plane 9 pode ignorar o primeiro)
sendPOSI([ll(1), ll(2), h0, -7, 0, hdg_deg_init, 1], 0, GlobalSocket);
pause(0.1);
sendPOSI([ll(1), ll(2), h0, -7, 0, hdg_deg_init, 1], 0, GlobalSocket);

sendDREF('sim/flightmodel/position/local_vx', VT0*sin(hdg_rad_init), GlobalSocket);
sendDREF('sim/flightmodel/position/local_vy', 0, GlobalSocket);
sendDREF('sim/flightmodel/position/local_vz', -VT0*cos(hdg_rad_init), GlobalSocket);
sendCTRL([0, 0, 0, 0.49, -998, -998], 0, GlobalSocket);

pause(0.3);
pauseSim(0, GlobalSocket);
pause(0.5);  % esperar X-Plane estabilizar

% Verificar posicionamento
s_check = double(getDREFs({'sim/flightmodel/position/elevation'}, GlobalSocket));
fprintf('\n  Aeronave posicionada: h_cmd=%.0fm, h_real=%.1fm, VT=%.0fm/s, hdg=%.0f deg\n', ...
    h0, s_check(1), VT0, hdg_deg_init);
if s_check(1) < h0 - 20
    fprintf('  AVISO: Altitude real (%.1fm) muito abaixo do esperado (%.0fm)!\n', s_check(1), h0);
    fprintf('  Tente reposicionar a aeronave manualmente no X-Plane e rodar novamente.\n');
end

%% ========== Loop do autopiloto com guiagem ==========
clear read_xplane;

timeout = 600;  % 10 min max
N = round(timeout / Ts);

% PIDs
pid_alt   = [0, 0, 0];
pid_theta = [0, 0, 0];
pid_vel   = [0, 0, 0];
pid_phi   = [0, 0, 0];
r_prev    = 0;

% Guiagem
wp_idx = 2;  % inicia mirando WP2 (WP1 = posicao inicial)
wp_idx_prev = wp_idx;
missao_completa = false;

% Logs
log_t     = zeros(N, 1);
log_VT    = zeros(N, 1);
log_h     = zeros(N, 1);
log_theta = zeros(N, 1);
log_phi   = zeros(N, 1);
log_psi   = zeros(N, 1);
log_xN    = zeros(N, 1);
log_xE    = zeros(N, 1);
log_wp    = zeros(N, 1);
log_de    = zeros(N, 1);
log_da    = zeros(N, 1);
log_dr    = zeros(N, 1);
log_dT    = zeros(N, 1);

fprintf('\n  Iniciando guiagem... (Ctrl+C para parar)\n\n');

k_final = N;
for k = 1:N
    tic;

    % Ler sensores
    s = read_xplane(0);
    VT_m=s(1); theta_m=s(2); q_m=s(3); h_m=s(4);
    phi_m=s(5); p_m=s(6); psi_m=s(7); r_m=s(8);
    xN_m=s(9); xE_m=s(10);

    % === GUIAGEM ===
    [psi_ref, h_ref, v_ref, wp_idx, dist] = ...
        guidance_star(xN_m, xE_m, WPs, R_accept, wp_idx);

    % Notificar troca de WP
    if wp_idx ~= wp_idx_prev
        fprintf('  [t=%5.1fs] WP%d alcancado (dist=%.0fm). Mirando WP%d (h=%.0fm, VT=%.0fm/s)\n', ...
            (k-1)*Ts, wp_idx_prev, dist, wp_idx, WPs(wp_idx,3), WPs(wp_idx,4));
        wp_idx_prev = wp_idx;
    end

    % Verificar missao completa (ultimo WP alcancado)
    if wp_idx == size(WPs, 1) && dist <= R_accept
        fprintf('  [t=%5.1fs] MISSAO COMPLETA! Ultimo WP alcancado (dist=%.0fm)\n', ...
            (k-1)*Ts, dist);
        k_final = k;
        missao_completa = true;
    end

    % === LONGITUDINAL ===
    e_alt = h_ref - h_m;
    [theta_cmd, pid_alt] = pid_step(e_alt, C_alt, Ts, pid_alt, [-0.17, 0.26]);

    e_theta = theta_cmd - theta_m;
    [de_pid, pid_theta] = pid_step(e_theta, C_theta, Ts, pid_theta, [-0.4363, 0.4363]);
    delta_e = max(-0.4363, min(0.4363, de_pid - Kq * q_m));

    e_vel = v_ref - VT_m;
    [dt_pid, pid_vel] = pid_step(e_vel, C_vel, Ts, pid_vel, [-0.51, 0.51]);
    delta_T = max(0, min(1, dt_pid + Ue(1)));

    % === LATERAL ===
    e_psi = atan2(sin(psi_ref - psi_m), cos(psi_ref - psi_m));
    phi_cmd = 0.3 * e_psi;

    e_phi = phi_cmd - phi_m;
    [da_pid, pid_phi] = pid_step(e_phi, C_phi, Ts, pid_phi, [-0.4363, 0.4363]);
    delta_a = max(-0.4363, min(0.4363, da_pid - Kp_sas * p_m));

    % Yaw washout
    r_washed = r_m - r_prev * exp(-Ts);
    r_prev = r_m;
    delta_r = max(-0.4363, min(0.4363, Kr * r_washed));

    % Enviar comandos
    send_xplane([delta_e, delta_a, delta_r, delta_T]);

    % Log
    log_t(k)     = (k-1)*Ts;
    log_VT(k)    = VT_m;
    log_h(k)     = h_m;
    log_theta(k) = theta_m;
    log_phi(k)   = phi_m;
    log_psi(k)   = psi_m;
    log_xN(k)    = xN_m;
    log_xE(k)    = xE_m;
    log_wp(k)    = wp_idx;
    log_de(k)    = delta_e;
    log_da(k)    = delta_a;
    log_dr(k)    = delta_r;
    log_dT(k)    = delta_T;

    % Manter sample time
    elapsed = toc;
    if elapsed < Ts
        pause(Ts - elapsed);
    end

    % Print progresso a cada 10s
    if mod(k, round(10/Ts)) == 0
        fprintf('  [t=%5.1fs] WP%d, dist=%.0fm, h=%.1fm, VT=%.1fm/s, psi=%.1f deg\n', ...
            log_t(k), wp_idx, dist, h_m, VT_m, rad2deg(psi_m));
    end

    if missao_completa
        break;
    end
end

%% ========== Encerrar ==========
try close_xplane; catch; end

% Cortar logs no tamanho real
log_t     = log_t(1:k_final);
log_VT    = log_VT(1:k_final);
log_h     = log_h(1:k_final);
log_theta = log_theta(1:k_final);
log_phi   = log_phi(1:k_final);
log_psi   = log_psi(1:k_final);
log_xN    = log_xN(1:k_final);
log_xE    = log_xE(1:k_final);
log_wp    = log_wp(1:k_final);
log_de    = log_de(1:k_final);
log_da    = log_da(1:k_final);
log_dr    = log_dr(1:k_final);
log_dT    = log_dT(1:k_final);

%% ========== Resultados ==========
fprintf('\n=== RESULTADO DA GUIAGEM ===\n');
fprintf('  Duracao: %.1f s (%d amostras)\n', log_t(end), k_final);
fprintf('  WPs visitados: %d de %d\n', max(log_wp) - 1, size(WPs, 1) - 1);
if missao_completa
    fprintf('  Status: MISSAO COMPLETA\n');
else
    fprintf('  Status: TIMEOUT (nao completou todos os WPs)\n');
end

fprintf('\n  Posicao final: N=%.1f E=%.1f h=%.1f m\n', ...
    log_xN(end), log_xE(end), log_h(end));
fprintf('  Distancia total: %.0f m\n', ...
    sum(sqrt(diff(log_xN).^2 + diff(log_xE).^2)));

%% ========== Preparar dados para plot3d_voo ==========
% Criar variavel POS no workspace (formato timeseries para plot3d_voo)
POS = timeseries([log_xN, log_xE, log_h], log_t);
POS.Name = 'POS';
assignin('base', 'POS', POS);

fprintf('\n  Dados salvos em POS (workspace). Rode plot3d_voo para visualizar.\n');
fprintf('\n=== GUIAGEM CONCLUIDA ===\n\n');


%% ========== Funcao PID discreta ==========
function [u, state] = pid_step(e, C, Ts, state, lims)
    integ   = state(1);
    e_prev  = state(2);
    df_prev = state(3);

    % Integral (trapezoidal)
    integ = integ + C.Ki * Ts * (e + e_prev) / 2;

    % Derivative (filtered)
    if C.Kd ~= 0
        alpha = C.N * Ts / (1 + C.N * Ts);
        df = alpha * C.Kd * (e - e_prev) / Ts + (1 - alpha) * df_prev;
    else
        df = 0;
    end

    % Output
    u = C.Kp * e + integ + df;

    % Anti-windup (clamping)
    if u > lims(2)
        u = lims(2);
        if e > 0, integ = state(1); end
    elseif u < lims(1)
        u = lims(1);
        if e < 0, integ = state(1); end
    end

    state = [integ, e, df];
end
