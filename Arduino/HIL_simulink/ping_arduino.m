function ping_arduino()
% Teste rapido do link serial com o Arduino Mega via hil_serial_step.
% Envia 5 frames com sensores de trim e imprime as 4 saidas U recebidas.
% Espera Ue ~ [0.491, 0.015, 0, 0] (sem erro de tracking).

fprintf('\n=== ping_arduino: testando link COM4 ===\n');

% Sensores de trim aproximados (estado de equilibrio):
% [p q r u v w phi theta psi VT alpha beta xN xE alt]
% Vamos mandar tudo zero exceto u=15 m/s, theta=0.0207 (~1.18 deg), alt=100, VT=15
sensors = zeros(1,15,'single');
sensors(4)  = single(15);      % u
sensors(8)  = single(0.0207);  % theta_eq
sensors(10) = single(15);      % VT
sensors(15) = single(100);     % alt -> sem erro vs h_ref hardcoded

clear hil_serial_step;
addpath(fileparts(mfilename('fullpath')));

for k = 1:5
    t0 = tic;
    U = hil_serial_step(sensors);
    dt = toc(t0)*1000;
    fprintf('passo %d (%.1f ms):  thr=%+.4f  elev=%+.4f  ail=%+.4f  rud=%+.4f\n', ...
        k, dt, U(1), U(2), U(3), U(4));
end

clear hil_serial_step;  % fecha porta
fprintf('=== link OK ===\n');
end
