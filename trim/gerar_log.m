%% Salvar xE, xN, xU após simulação e log para cada caso:

 t_m    =   out.Y.time;             
 da_m   =   out.U.signals.values(:,3) ; 
 dr_m   =   out.U.signals.values(:,4) ; 
 dt_m   =   out.U.signals.values(:,1) ; 
 VT_m   =   out.Y.signals.values(:,1) ; 
 beta_m =   out.Y.signals.values(:,3) ; 
 phi_m  =   out.Y.signals.values(:,7) ; 
 p_m    =   out.Y.signals.values(:,4) ; 
 r_m    =   out.Y.signals.values(:,6) ; 
 xN     =   out.Y.signals.values(:,10); 
 xE     =   out.Y.signals.values(:,11); 
 xU     =   out.Y.signals.values(:,12); %%valor de -xD

 pdot_m =   out.Y.signals.values(:,13); 
 rdot_m =   out.Y.signals.values(:,14); 
 ay_m   =   out.Y.signals.values(:,15); 



%% Para gerar log para teste no OEM

 OEM_Data_lat_Medido_comp = [t_m da_m dr_m beta_m ...
    phi_m p_m r_m pdot_m rdot_m ay_m];
save('log_Simulado_manobra5_AVL.mat', 'OEM_Data_lat_Medido_comp');

disp('log salvo');

%% salvar variaveis direto na planilha
% Modelo 
% save('log_Simulado_manobra2_Modelo1.mat', 't_m', 'da_m', 'dr_m', 'beta_m', 'phi_m',...
%  'p_m', 'r_m', 'pdot_m', 'rdot_m', 'ay_m', 'xE', 'xN', 'xU');


% 
% Caso T1
% save('neu_T1.mat', 'xE', 'xN', 'xU');
% 
% % Caso T2
% save('neu_T2.mat', 'xE', 'xN', 'xU');

% % Caso T3
% save('neu_T3.mat', 'xE', 'xN', 'xU');

% % Caso T4
% save('neu_T4.mat', 'xE', 'xN', 'xU');

% Caso Original
% save('neu_Original.mat', 'xE', 'xN', 'xU');


 

figure;
plot3(xE, xN, xU, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Leste (xE) [m]');
ylabel('Norte (xN) [m]');
zlabel('Altitude [m]');
title('Trajetória 3D da Aeronave em Coordenadas Locais');


figure;

subplot(3,2,1);
plot(t_m, da_m, 'k', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\delta_a [rad]');
title('Manobra no Aileron'); grid on;

subplot(3,2,2);
plot(t_m, dr_m, 'k', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\delta_r [rad]');
title('Manobra no Leme'); grid on;

subplot(3,2,3);
plot(t_m, rad2deg(p_m), 'r', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('p [°/s]');
title('Taxa de Rolagem'); grid on;

subplot(3,2,4);
plot(t_m, rad2deg(r_m), 'm', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('r [°/s]');
title('Taxa de Guinada'); grid on;

subplot(3,2,5);
plot(t_m, rad2deg(phi_m), 'g', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\phi [°]');
title('Ângulo de Rolamento'); grid on;

subplot(3,2,6);
plot(t_m, rad2deg(beta_m), 'b', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\beta [°]');
title('Ângulo de Deslizamento Lateral'); grid on;



%--------------------------------------------------------
%% manobra e ângulos no mesmo gráfico:
% % Supondo que você já tenha carregado ou executado gerar_log.m
% % e que as variáveis a seguir estejam disponíveis:
% % t_m, da_m, dr_m, phi_m, beta_m
% 
% % ==== GRÁFICO 1: Manobras Laterais ====
% figure('Name','Manobras Laterais');
% 
% plot(t_m, da_m, 'k-', 'LineWidth', 1.5); hold on;
% plot(t_m, dr_m, 'b--', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Deflexão [rad]');
% title('Manobras Laterais: Aileron e Leme');
% legend({'\delta_a (aileron)', '\delta_r (leme)'}, 'Location', 'best');
% grid on;
% 
% % ==== GRÁFICO 2: Ângulos Laterais ====
% figure('Name','Ângulos Laterais');
% 
% plot(t_m, rad2deg(phi_m), 'r-', 'LineWidth', 1.5); hold on;
% plot(t_m, rad2deg(beta_m), 'g--', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Ângulo [°]');
% title('Ângulos Laterais: Rolamento e Deslizamento');
% legend({'\phi (rolamento)', '\beta (deslizamento)'}, 'Location', 'best');
% grid on;

%% com o ângulo e a manobra na mesma figura:
% figure('Name', 'Manobra e Ângulos');
% 
% % ==== SUBPLOT 1: Manobras (δa e δr) ====
% subplot(2,1,1);
% plot(t_m, da_m, 'k-', 'LineWidth', 1.5); hold on;
% plot(t_m, dr_m, 'b--', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Deflexão [rad]');
% title('Manobra');
% legend({'\delta_a (aileron)', '\delta_r (leme)'}, 'Location', 'best');
% grid on;
% 
% % ==== SUBPLOT 2: Ângulos (ϕ e β) ====
% subplot(2,1,2);
% plot(t_m, rad2deg(phi_m), 'r-', 'LineWidth', 1.5); hold on;
% plot(t_m, rad2deg(beta_m), 'g--', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Ângulo [°]');
% title('Ângulos: Rolamento e Deslizamento');
% legend({'\phi (rolamento)', '\beta (deslizamento)'}, 'Location', 'best');
% grid on;
% 
% saveas(gcf, 'manobra_validar.png');
