%% =========================================================================
%  RODAR MODELO LINEAR - Piper J-3 Cub 1/6
%  Analise completa do modelo linearizado (longitudinal + latero-direcional)
%  =========================================================================
clear; clc; close all;

%% 1. Carregar matrizes do espaco de estados
MATRIZES;

fprintf('========================================\n');
fprintf(' MODELO LINEAR - Piper J-3 Cub 1/6\n');
fprintf('========================================\n\n');

%% 2. Definir nomes dos estados, entradas e saidas

% Longitudinal
estados_long = {'U [m/s]', 'W [m/s]', 'q [rad/s]', '\theta [rad]', 'h [m]'};
entradas_long = {'\delta_T', '\delta_e'};
saidas_long = {'U [m/s]', 'W [m/s]', 'q [rad/s]', '\theta [rad]', 'h [m]'};

% Latero-direcional
estados_lat = {'V [m/s]', 'p [rad/s]', 'r [rad/s]', '\phi [rad]', '\psi [rad]'};
entradas_lat = {'\delta_a', '\delta_r'};
saidas_lat = {'\beta [rad]', 'p [rad/s]', 'r [rad/s]', '\phi [rad]', '\psi [rad]'};

%% 3. Criar sistemas de espaco de estados
sys_long = ss(A_long, B_long, C_long, D_long, ...
    'StateName', estados_long, ...
    'InputName', entradas_long, ...
    'OutputName', saidas_long);

sys_lat = ss(A_lat, B_lat, C_lat, D_lat, ...
    'StateName', estados_lat, ...
    'InputName', entradas_lat, ...
    'OutputName', saidas_lat);

%% 4. Analise de autovalores - LONGITUDINAL
fprintf('--- LONGITUDINAL ---\n');
fprintf('Ponto de equilibrio: U=%.2f m/s, W=%.4f m/s, q=%.4f rad/s, theta=%.4f rad, h=%.1f m\n', ...
    Xe_long(1), Xe_long(2), Xe_long(3), Xe_long(4), Xe_long(5));
fprintf('\n');

eig_long = eig(A_long);
fprintf('Autovalores longitudinais:\n');
for i = 1:length(eig_long)
    if imag(eig_long(i)) >= 0
        if imag(eig_long(i)) == 0
            fprintf('  lambda_%d = %.4f (real)\n', i, real(eig_long(i)));
        else
            fprintf('  lambda_%d = %.4f +/- %.4fj\n', i, real(eig_long(i)), abs(imag(eig_long(i))));
        end
    end
end

% Calcular wn e zeta
[wn_long, zeta_long] = damp(sys_long);
fprintf('\n  Modo             | wn [rad/s] | zeta    | Periodo [s]\n');
fprintf('  -----------------|------------|---------|------------\n');

% Identificar modos (pares conjugados e reais)
idx_processado = false(size(wn_long));
for i = 1:length(wn_long)
    if idx_processado(i), continue; end
    % Procurar par conjugado
    par = find(abs(wn_long - wn_long(i)) < 0.01 & abs(zeta_long - zeta_long(i)) < 0.01 & ~idx_processado);
    if length(par) >= 2
        idx_processado(par(1:2)) = true;
        if wn_long(par(1)) > 1
            nome = 'Periodo Curto';
        else
            nome = 'Fugoide      ';
        end
        T = 2*pi / (wn_long(par(1)) * sqrt(1 - zeta_long(par(1))^2));
        fprintf('  %s | %10.3f | %7.4f | %10.2f\n', nome, wn_long(par(1)), zeta_long(par(1)), T);
    else
        idx_processado(par(1)) = true;
        fprintf('  Real             | %10.3f | %7.4f |        -\n', wn_long(par(1)), zeta_long(par(1)));
    end
end

%% 5. Analise de autovalores - LATERO-DIRECIONAL
fprintf('\n--- LATERO-DIRECIONAL ---\n');
fprintf('Ponto de equilibrio: V=%.2f, p=%.4f, r=%.4f, phi=%.4f, psi=%.4f\n', ...
    Xe_lat(1), Xe_lat(2), Xe_lat(3), Xe_lat(4), Xe_lat(5));
fprintf('\n');

eig_lat = eig(A_lat);
fprintf('Autovalores latero-direcionais:\n');
for i = 1:length(eig_lat)
    if imag(eig_lat(i)) >= 0
        if imag(eig_lat(i)) == 0
            fprintf('  lambda_%d = %.4f (real)\n', i, real(eig_lat(i)));
        else
            fprintf('  lambda_%d = %.4f +/- %.4fj\n', i, real(eig_lat(i)), abs(imag(eig_lat(i))));
        end
    end
end

[wn_lat, zeta_lat] = damp(sys_lat);
fprintf('\n  Modo             | wn [rad/s] | zeta    | Periodo [s]\n');
fprintf('  -----------------|------------|---------|------------\n');
idx_processado = false(size(wn_lat));
for i = 1:length(wn_lat)
    if idx_processado(i), continue; end
    par = find(abs(wn_lat - wn_lat(i)) < 0.01 & abs(zeta_lat - zeta_lat(i)) < 0.01 & ~idx_processado);
    if length(par) >= 2
        idx_processado(par(1:2)) = true;
        T = 2*pi / (wn_lat(par(1)) * sqrt(max(1 - zeta_lat(par(1))^2, eps)));
        fprintf('  Dutch Roll       | %10.3f | %7.4f | %10.2f\n', wn_lat(par(1)), zeta_lat(par(1)), T);
    else
        idx_processado(par(1)) = true;
        ev = eig_lat(abs(eig_lat - (-wn_lat(par(1))*zeta_lat(par(1)))) < 0.1);
        if ~isempty(ev) && real(ev(1)) > 0
            nome = 'Espiral (INST)';
        elseif wn_lat(par(1)) > 10
            nome = 'Rolamento Puro';
        else
            nome = 'Real          ';
        end
        fprintf('  %s | %10.3f | %7.4f |        -\n', nome, wn_lat(par(1)), zeta_lat(par(1)));
    end
end

%% 6. Estabilidade
fprintf('\n--- ESTABILIDADE ---\n');
if all(real(eig_long) < 0)
    fprintf('  Longitudinal: ESTAVEL (todos autovalores com parte real negativa)\n');
else
    fprintf('  Longitudinal: INSTAVEL!\n');
end
if all(real(eig_lat) < 0)
    fprintf('  Latero-direcional: ESTAVEL\n');
else
    fprintf('  Latero-direcional: INSTAVEL (modo espiral positivo)\n');
end

%% 7. Resposta ao degrau - LONGITUDINAL
t_sim = 0:0.01:30; % 30 segundos

% Degrau no profundor (delta_e = 1 grau = 0.0175 rad)
delta_e_step = deg2rad(1);
u_long = [zeros(length(t_sim), 1), delta_e_step * ones(length(t_sim), 1)];
[y_long_de, t_long_de] = lsim(sys_long, u_long, t_sim);

% Somar ponto de equilibrio nas saidas
y_long_de(:,1) = y_long_de(:,1) + Xe_long(1); % U
y_long_de(:,2) = y_long_de(:,2) + Xe_long(2); % W
y_long_de(:,4) = y_long_de(:,4) + Xe_long(4); % theta
y_long_de(:,5) = y_long_de(:,5) + Xe_long(5); % h

% Degrau na manete (delta_T = 0.1)
delta_T_step = 0.1;
u_long_dt = [delta_T_step * ones(length(t_sim), 1), zeros(length(t_sim), 1)];
[y_long_dt, t_long_dt] = lsim(sys_long, u_long_dt, t_sim);
y_long_dt(:,1) = y_long_dt(:,1) + Xe_long(1);
y_long_dt(:,2) = y_long_dt(:,2) + Xe_long(2);
y_long_dt(:,4) = y_long_dt(:,4) + Xe_long(4);
y_long_dt(:,5) = y_long_dt(:,5) + Xe_long(5);

%% 8. Resposta ao degrau - LATERO-DIRECIONAL
% Degrau no aileron (delta_a = 1 grau)
delta_a_step = deg2rad(1);
u_lat_da = [delta_a_step * ones(length(t_sim), 1), zeros(length(t_sim), 1)];
[y_lat_da, t_lat_da] = lsim(sys_lat, u_lat_da, t_sim);

% Degrau no leme (delta_r = 1 grau)
delta_r_step = deg2rad(1);
u_lat_dr = [zeros(length(t_sim), 1), delta_r_step * ones(length(t_sim), 1)];
[y_lat_dr, t_lat_dr] = lsim(sys_lat, u_lat_dr, t_sim);

%% 9. GRAFICOS

% --- Figura 1: Autovalores ---
figure('Name', 'Autovalores', 'Position', [50 50 900 400]);

subplot(1,2,1);
plot(real(eig_long), imag(eig_long), 'bx', 'MarkerSize', 12, 'LineWidth', 2);
hold on;
xline(0, 'k--');
yline(0, 'k--');
grid on;
xlabel('Parte Real'); ylabel('Parte Imaginaria');
title('Autovalores Longitudinais');
for i = 1:length(eig_long)
    if imag(eig_long(i)) >= 0
        text(real(eig_long(i))+0.1, imag(eig_long(i))+0.15, ...
            sprintf('%.2f%+.2fj', real(eig_long(i)), imag(eig_long(i))), 'FontSize', 8);
    end
end

subplot(1,2,2);
plot(real(eig_lat), imag(eig_lat), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
hold on;
xline(0, 'k--');
yline(0, 'k--');
grid on;
xlabel('Parte Real'); ylabel('Parte Imaginaria');
title('Autovalores Latero-Direcionais');
for i = 1:length(eig_lat)
    if imag(eig_lat(i)) >= 0
        text(real(eig_lat(i))+0.3, imag(eig_lat(i))+0.15, ...
            sprintf('%.2f%+.2fj', real(eig_lat(i)), imag(eig_lat(i))), 'FontSize', 8);
    end
end
sgtitle('Mapa de Polos');

% --- Figura 2: Resposta longitudinal ao degrau no profundor ---
figure('Name', 'Resposta Long - delta_e', 'Position', [100 100 1000 600]);
sgtitle(sprintf('Resposta Longitudinal - Degrau de %.1f grau no profundor', rad2deg(delta_e_step)));

subplot(2,3,1);
plot(t_long_de, y_long_de(:,1), 'b', 'LineWidth', 1.2);
yline(Xe_long(1), 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('U [m/s]');
title('Velocidade U'); grid on;

subplot(2,3,2);
plot(t_long_de, y_long_de(:,2), 'b', 'LineWidth', 1.2);
yline(Xe_long(2), 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('W [m/s]');
title('Velocidade W'); grid on;

subplot(2,3,3);
plot(t_long_de, rad2deg(y_long_de(:,3)), 'b', 'LineWidth', 1.2);
yline(0, 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('q [deg/s]');
title('Taxa de arfagem q'); grid on;

subplot(2,3,4);
plot(t_long_de, rad2deg(y_long_de(:,4)), 'b', 'LineWidth', 1.2);
yline(rad2deg(Xe_long(4)), 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('\theta [deg]');
title('Angulo de arfagem \theta'); grid on;

subplot(2,3,5);
plot(t_long_de, y_long_de(:,5), 'b', 'LineWidth', 1.2);
yline(Xe_long(5), 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('h [m]');
title('Altitude h'); grid on;

subplot(2,3,6);
% Calcular alpha e VT a partir de U e W
VT_de = sqrt(y_long_de(:,1).^2 + y_long_de(:,2).^2);
alpha_de = rad2deg(atan2(y_long_de(:,2), y_long_de(:,1)));
plot(t_long_de, VT_de, 'b', 'LineWidth', 1.2);
yline(15, 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('V_T [m/s]');
title('Velocidade Total V_T'); grid on;

% --- Figura 3: Resposta longitudinal ao degrau na manete ---
figure('Name', 'Resposta Long - delta_T', 'Position', [150 50 1000 600]);
sgtitle(sprintf('Resposta Longitudinal - Degrau de %.1f%% na manete', delta_T_step*100));

subplot(2,3,1);
plot(t_long_dt, y_long_dt(:,1), 'r', 'LineWidth', 1.2);
yline(Xe_long(1), 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('U [m/s]');
title('Velocidade U'); grid on;

subplot(2,3,2);
plot(t_long_dt, y_long_dt(:,2), 'r', 'LineWidth', 1.2);
yline(Xe_long(2), 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('W [m/s]');
title('Velocidade W'); grid on;

subplot(2,3,3);
plot(t_long_dt, rad2deg(y_long_dt(:,3)), 'r', 'LineWidth', 1.2);
yline(0, 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('q [deg/s]');
title('Taxa de arfagem q'); grid on;

subplot(2,3,4);
plot(t_long_dt, rad2deg(y_long_dt(:,4)), 'r', 'LineWidth', 1.2);
yline(rad2deg(Xe_long(4)), 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('\theta [deg]');
title('Angulo de arfagem \theta'); grid on;

subplot(2,3,5);
plot(t_long_dt, y_long_dt(:,5), 'r', 'LineWidth', 1.2);
yline(Xe_long(5), 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('h [m]');
title('Altitude h'); grid on;

subplot(2,3,6);
VT_dt = sqrt(y_long_dt(:,1).^2 + y_long_dt(:,2).^2);
plot(t_long_dt, VT_dt, 'r', 'LineWidth', 1.2);
yline(15, 'k--', 'Equilibrio');
xlabel('Tempo [s]'); ylabel('V_T [m/s]');
title('Velocidade Total V_T'); grid on;

% --- Figura 4: Resposta latero-direcional ao degrau no aileron ---
figure('Name', 'Resposta Lat - delta_a', 'Position', [200 100 1000 500]);
sgtitle(sprintf('Resposta Latero-Direcional - Degrau de %.1f grau no aileron', rad2deg(delta_a_step)));

subplot(2,3,1);
plot(t_lat_da, rad2deg(y_lat_da(:,1)), 'b', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\beta [deg]');
title('Angulo de derrapagem \beta'); grid on;

subplot(2,3,2);
plot(t_lat_da, rad2deg(y_lat_da(:,2)), 'b', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('p [deg/s]');
title('Taxa de rolamento p'); grid on;

subplot(2,3,3);
plot(t_lat_da, rad2deg(y_lat_da(:,3)), 'b', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('r [deg/s]');
title('Taxa de guinada r'); grid on;

subplot(2,3,4);
plot(t_lat_da, rad2deg(y_lat_da(:,4)), 'b', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\phi [deg]');
title('Angulo de rolamento \phi'); grid on;

subplot(2,3,5);
plot(t_lat_da, rad2deg(y_lat_da(:,5)), 'b', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\psi [deg]');
title('Angulo de guinada \psi'); grid on;

% --- Figura 5: Resposta latero-direcional ao degrau no leme ---
figure('Name', 'Resposta Lat - delta_r', 'Position', [250 50 1000 500]);
sgtitle(sprintf('Resposta Latero-Direcional - Degrau de %.1f grau no leme', rad2deg(delta_r_step)));

subplot(2,3,1);
plot(t_lat_dr, rad2deg(y_lat_dr(:,1)), 'r', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\beta [deg]');
title('Angulo de derrapagem \beta'); grid on;

subplot(2,3,2);
plot(t_lat_dr, rad2deg(y_lat_dr(:,2)), 'r', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('p [deg/s]');
title('Taxa de rolamento p'); grid on;

subplot(2,3,3);
plot(t_lat_dr, rad2deg(y_lat_dr(:,3)), 'r', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('r [deg/s]');
title('Taxa de guinada r'); grid on;

subplot(2,3,4);
plot(t_lat_dr, rad2deg(y_lat_dr(:,4)), 'r', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\phi [deg]');
title('Angulo de rolamento \phi'); grid on;

subplot(2,3,5);
plot(t_lat_dr, rad2deg(y_lat_dr(:,5)), 'r', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\psi [deg]');
title('Angulo de guinada \psi'); grid on;

% --- Figura 6: Diagrama de Bode ---
figure('Name', 'Bode Longitudinal', 'Position', [300 100 900 500]);
bode(sys_long(1,2), sys_long(3,2), sys_long(4,2));
legend('U/\delta_e', 'q/\delta_e', '\theta/\delta_e');
title('Bode - Funcoes de transferencia longitudinais (entrada: \delta_e)');
grid on;

figure('Name', 'Bode Latero-direcional', 'Position', [350 50 900 500]);
bode(sys_lat(1,1), sys_lat(2,1), sys_lat(4,1));
legend('\beta/\delta_a', 'p/\delta_a', '\phi/\delta_a');
title('Bode - Funcoes de transferencia latero-direcionais (entrada: \delta_a)');
grid on;

%% 10. Resumo final
fprintf('\n========================================\n');
fprintf(' RESUMO DA ANALISE\n');
fprintf('========================================\n');
fprintf(' Modelo longitudinal: %d estados, %d entradas, %d saidas\n', size(A_long,1), size(B_long,2), size(C_long,1));
fprintf(' Modelo latero-dir.:  %d estados, %d entradas, %d saidas\n', size(A_lat,1), size(B_lat,2), size(C_lat,1));
fprintf(' VT equilibrio: %.1f m/s\n', sqrt(Xe_long(1)^2 + Xe_long(2)^2));
fprintf(' Theta equilibrio: %.2f deg\n', rad2deg(Xe_long(4)));
fprintf(' Altitude equilibrio: %.0f m\n', -Xe_long(5));
fprintf('\n 7 figuras geradas com sucesso.\n');
fprintf('========================================\n');
