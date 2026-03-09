%% teste_autopiloto.m - Teste do autopiloto em malha fechada com X-Plane
% Posiciona a aeronave em voo e roda o autopiloto por 60 segundos.
%
% Pre-requisito: inicializar_xplane
%
% Uso:
%   >> inicializar_xplane
%   >> teste_autopiloto

import XPlaneConnect.*;

global GlobalSocket;
if isempty(GlobalSocket)
    GlobalSocket = openUDP('127.0.0.1', 49009);
end

%% ========== Posicionar aeronave em voo ==========
drefs_ll = {'sim/flightmodel/position/latitude', 'sim/flightmodel/position/longitude'};
ll = double(getDREFs(drefs_ll, GlobalSocket));

pauseSim(1, GlobalSocket);
pause(0.2);

hdg = 0;  % heading Norte (alinhado com psi_ref=0)
sendPOSI([ll(1), ll(2), 100, -7, 0, hdg, 1], 0, GlobalSocket);

VT0 = 15; hdg_rad = hdg * pi/180;
sendDREF('sim/flightmodel/position/local_vx', VT0*sin(hdg_rad), GlobalSocket);
sendDREF('sim/flightmodel/position/local_vy', 0, GlobalSocket);
sendDREF('sim/flightmodel/position/local_vz', -VT0*cos(hdg_rad), GlobalSocket);
sendCTRL([0, 0, 0, 0.49, -998, -998], 0, GlobalSocket);

pause(0.2);
pauseSim(0, GlobalSocket);
disp('Aeronave posicionada: 100m, VT=15m/s, heading=0. Autopiloto ativado.');

%% ========== Loop do autopiloto ==========
clear read_xplane;  % limpar persistent vars

Ts = Ts_xplane;
duracao = 600;  % 10 minutos - parar com Ctrl+C
N = round(duracao / Ts);

% Estado dos PIDs: [integral, error_prev, deriv_filt_prev]
pid_alt   = [0, 0, 0];
pid_theta = [0, 0, 0];
pid_vel   = [0, 0, 0];
pid_phi   = [0, 0, 0];
r_prev    = 0;

log_t     = zeros(N, 1);
log_VT    = zeros(N, 1);
log_h     = zeros(N, 1);
log_theta = zeros(N, 1);
log_phi   = zeros(N, 1);
log_psi   = zeros(N, 1);
log_de    = zeros(N, 1);
log_dT    = zeros(N, 1);

for k = 1:N
    tic;

    % Ler sensores
    s = read_xplane(0);
    VT_m=s(1); theta_m=s(2); q_m=s(3); h_m=s(4);
    phi_m=s(5); p_m=s(6); psi_m=s(7); r_m=s(8);

    % === LONGITUDINAL ===
    % Altitude -> theta_ref
    e_alt = h_ref - h_m;
    [theta_cmd, pid_alt] = pid_step(e_alt, C_alt, Ts, pid_alt, [-0.17, 0.26]);

    % Theta -> elevator
    e_theta = theta_cmd - theta_m;
    [de_pid, pid_theta] = pid_step(e_theta, C_theta, Ts, pid_theta, [-0.4363, 0.4363]);
    delta_e = de_pid - Kq * q_m;
    delta_e = max(-0.4363, min(0.4363, delta_e));

    % Velocidade -> throttle
    e_vel = VT_ref - VT_m;
    [dt_pid, pid_vel] = pid_step(e_vel, C_vel, Ts, pid_vel, [-0.51, 0.51]);
    delta_T = dt_pid + Ue(1);
    delta_T = max(0, min(1, delta_T));

    % === LATERAL ===
    e_psi = atan2(sin(psi_ref - psi_m), cos(psi_ref - psi_m));
    phi_cmd = 0.3 * e_psi;

    e_phi = phi_cmd - phi_m;
    [da_pid, pid_phi] = pid_step(e_phi, C_phi, Ts, pid_phi, [-0.4363, 0.4363]);
    delta_a = da_pid - Kp_sas * p_m;
    delta_a = max(-0.4363, min(0.4363, delta_a));

    % Yaw washout
    r_washed = r_m - r_prev * exp(-Ts);
    r_prev = r_m;
    delta_r = Kr * r_washed;
    delta_r = max(-0.4363, min(0.4363, delta_r));

    % Enviar comandos
    send_xplane([delta_e, delta_a, delta_r, delta_T]);

    % Log
    log_t(k)     = (k-1)*Ts;
    log_VT(k)    = VT_m;
    log_h(k)     = h_m;
    log_theta(k) = rad2deg(theta_m);
    log_phi(k)   = rad2deg(phi_m);
    log_psi(k)   = rad2deg(psi_m);
    log_de(k)    = delta_e;
    log_dT(k)    = delta_T;

    % Manter sample time
    elapsed = toc;
    if elapsed < Ts
        pause(Ts - elapsed);
    end
end

%% ========== Resultados ==========
disp(' ');
disp('=== RESULTADO DO AUTOPILOTO ===');
disp('  t(s)   VT(m/s)   h(m)   theta(deg) phi(deg) psi(deg)');
for k = [1, round(N*0.1), round(N*0.2), round(N*0.4), round(N*0.6), round(N*0.8), N]
    fprintf('  %5.1f   %6.1f   %6.1f    %6.1f    %6.1f   %6.1f\n', ...
        log_t(k), log_VT(k), log_h(k), log_theta(k), log_phi(k), log_psi(k));
end
fprintf('\n  Ref: VT=%.1f m/s, h=%.1f m, psi=%.1f deg\n', VT_ref, h_ref, rad2deg(psi_ref));
fprintf('  Final: VT=%.1f m/s, h=%.1f m, theta=%.1f deg\n', log_VT(end), log_h(end), log_theta(end));

% Verificar estabilidade
dh = abs(log_h(end) - h_ref);
dVT = abs(log_VT(end) - VT_ref);
if dh < 20 && dVT < 5
    disp('  >> ESTAVEL: aeronave mantendo referencias!');
elseif log_h(end) < 10
    disp('  >> INSTAVEL: aeronave caiu! Verificar sinal do elevator.');
else
    disp('  >> PARCIAL: convergindo mas nao estabilizou em 10s.');
end

close_xplane;
disp('=== TESTE CONCLUIDO ===');


%% ========== Funcao PID discreta ==========
function [u, state] = pid_step(e, C, Ts, state, lims)
    integ   = state(1);
    e_prev  = state(2);
    df_prev = state(3);

    % Integral (trapezoidal)
    integ = integ + C.Ki * Ts * (e + e_prev) / 2;

    % Derivative (filtered, N = filter coeff)
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
