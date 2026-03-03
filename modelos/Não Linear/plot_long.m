%% Salvar após simulação para cada caso:

 t_m    =   out.Y.time;             
 dt_m   =   out.U.signals.values(:,1) ; 
 de_m   =   out.U.signals.values(:,2) ;
 
 VT_m   =   out.Y.signals.values(:,1) ;
 beta_m =   out.Y.signals.values(:,3) ; 

 theta_m  =   out.Y.signals.values(:,8) ; 
 q_m    =   out.Y.signals.values(:,5) ; 
 xN     =   out.Y.signals.values(:,10); 
 xE     =   out.Y.signals.values(:,11); 
 xU     =   out.Y.signals.values(:,12); %%valor de -xD

 ay_m   =   out.Y.signals.values(:,15); 

figure(1);
plot3(xE, xN, xU, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Leste (xE) [m]');
ylabel('Norte (xN) [m]');
zlabel('Altitude [m]');
title('Trajetória 3D da Aeronave em Coordenadas Locais');


figure(2);
subplot(2,2,1);
plot(t_m, dt_m, 'b', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\delta_t [rad]');
title('Manobra na Manete'); grid on;

subplot(2,2,2);
plot(t_m, de_m, 'b', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\delta_e [rad]');
title('Manobra no Profundor'); grid on;

subplot(2,2,3);
plot(t_m, rad2deg(q_m), 'r', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('q [°/s]');
title('Taxa de Arfagem'); grid on;

subplot(2,2,4);
plot(t_m, rad2deg(theta_m), 'g', 'LineWidth', 1.2);
xlabel('Tempo [s]'); ylabel('\theta [°]');
title('Ângulo de Arfagem'); grid on;

