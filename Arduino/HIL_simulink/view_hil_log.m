function view_hil_log()
% view_hil_log - Plota o conteudo de hil_log.csv (gerado por
% hil_serial_step durante a sim HIL).
%
% Mostra os 15 sensores enviados ao Mega e os 4 atuadores recebidos,
% organizados em subplots tematicos. O CSV e' atualizado a cada nova sim.

log_path = fullfile(fileparts(mfilename('fullpath')), 'hil_log.csv');
if ~isfile(log_path)
    error('view_hil_log:noFile', ...
        'Nao encontrei %s. Rode uma sim HIL antes (run_hil_test ou modelo).', log_path);
end

T = readtable(log_path);
n = height(T);
if n == 0
    error('view_hil_log:empty', 'hil_log.csv esta vazio.');
end
fprintf('hil_log.csv: %d frames, t = %.2fs\n', n, max(T.t));

fig = figure('Name','HIL serial log','NumberTitle','off', ...
             'Position',[80 80 1300 850]);

% --- Sensores PC -> MEGA ---
subplot(3,2,1);
plot(T.t, T.alt, 'LineWidth',1.2); grid on;
title('alt (m) — enviado ao Mega'); xlabel('t (s)'); ylabel('m');

subplot(3,2,2);
plot(T.t, T.VT, 'LineWidth',1.2); grid on;
title('VT (m/s) — enviado ao Mega'); xlabel('t (s)');

subplot(3,2,3);
plot(T.t, T.phi*180/pi, T.t, T.theta*180/pi, T.t, T.psi*180/pi, 'LineWidth',1.0);
grid on; legend('\phi','\theta','\psi','Location','best');
title('Atitude (deg) — enviada ao Mega'); xlabel('t (s)');

subplot(3,2,4);
plot(T.t, T.p, T.t, T.q, T.t, T.r, 'LineWidth',1.0);
grid on; legend('p','q','r','Location','best');
title('Vel. angulares (rad/s) — enviadas ao Mega'); xlabel('t (s)');

% --- Atuadores MEGA -> PC ---
subplot(3,2,5);
plot(T.t, T.thr, 'LineWidth',1.2); grid on; ylim([0 1]);
title('thr — recebido do Mega'); xlabel('t (s)'); ylabel('throttle [0,1]');

subplot(3,2,6);
plot(T.t, T.elev*180/pi, T.t, T.ail*180/pi, T.t, T.rud*180/pi, 'LineWidth',1.0);
grid on; legend('elev','ail','rud','Location','best');
title('Superficies (deg) — recebidas do Mega'); xlabel('t (s)');

sgtitle(sprintf('hil\\_log.csv — %d frames @ 100 Hz', n));

% Estatisticas resumidas no Command Window
fprintf('\nResumo:\n');
fprintf('  alt:   %.2f -> %.2f m   (final %.4f, std %.4f)\n', ...
    T.alt(1), T.alt(end), T.alt(end), std(T.alt(end-min(n-1,1000):end)));
fprintf('  VT:    %.2f -> %.2f m/s (final %.4f, std %.4f)\n', ...
    T.VT(1), T.VT(end), T.VT(end), std(T.VT(end-min(n-1,1000):end)));
fprintf('  thr:   min %.4f, max %.4f, med %.4f\n', min(T.thr), max(T.thr), mean(T.thr));
fprintf('  elev:  min %+.4f, max %+.4f rad (%+.2f / %+.2f deg)\n', ...
    min(T.elev), max(T.elev), min(T.elev)*180/pi, max(T.elev)*180/pi);
fprintf('  ail:   min %+.4f, max %+.4f rad\n', min(T.ail), max(T.ail));
fprintf('  rud:   min %+.4f, max %+.4f rad\n', min(T.rud), max(T.rud));
end
