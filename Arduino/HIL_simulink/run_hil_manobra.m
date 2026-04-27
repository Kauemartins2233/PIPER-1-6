% run_hil_manobra  -  Testa o controlador HIL v1 com perturbacoes na CI.

cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));
clear hil_serial_step;
fprintf('--- inicializar ---\n');
inicializar;

repo     = pwd;
mdl      = 'modeloNL1_HIL';
mdl_path = fullfile(repo,'Arduino','HIL_simulink',[mdl '.slx']);
load_system(mdl_path);
set_param(mdl,'SignalLogging','on','SignalLoggingName','logsout', ...
    'StopTime','30','SaveOutput','on','SaveFormat','Dataset', ...
    'ReturnWorkspaceOutputs','on');

Xe_orig = Xe;

% Cada cenario: nome, indice em Xe a alterar, novo valor
%   Xe = [u v w p q r phi theta psi xN xE xD]   (1..12)
cen = { ...
    'Alt 110m -> 100m',  12, -110;        ...
    'VT 17 -> 15.11',     1,  17;          ...
    'Roll 20deg -> 0',    7,  20*pi/180;   ...
};

n = size(cen,1);
results = cell(n,1);

for k = 1:n
    Xe = Xe_orig;
    Xe(cen{k,2}) = cen{k,3};
    fprintf('\n==============================\n');
    fprintf('Cenario %d: %s\n', k, cen{k,1});
    fprintf('Xe perturbado: alt=%.2f m, VT=%.3f m/s, phi=%.2f deg\n', ...
        -Xe(12), norm(Xe(1:3)), Xe(7)*180/pi);
    fprintf('==============================\n');
    clear hil_serial_step;
    pause(3);   % deixa o Arduino terminar o reset DTR antes de reabrir COM
    t0 = tic;
    out = sim(mdl);
    fprintf('sim concluida em %.1f s wall-clock\n', toc(t0));

    ls = out.logsout;
    sig_alt = []; sig_vt = [];
    for i = 1:ls.numElements
        el = ls.getElement(i); nm = lower(el.Name);
        if isempty(sig_alt) && (contains(nm,'alt') || contains(nm,'h_'))
            sig_alt = el.Values;
        elseif isempty(sig_vt) && (contains(nm,'vt') || contains(nm,'vel') || contains(nm,'airspeed'))
            sig_vt = el.Values;
        end
    end

    s_alt = local_stats(sig_alt, 100);
    s_vt  = local_stats(sig_vt , 15.1117);

    if ~isempty(s_alt)
        fprintf('  Alt: ini=%.2f  min=%.2f  max=%.2f  final=%.4f  std10=%.4f  t_2%%=%.1fs  target=%.2f\n', ...
            s_alt.ini, s_alt.min, s_alt.max, s_alt.final, s_alt.std10, s_alt.ts, s_alt.target);
    end
    if ~isempty(s_vt)
        fprintf('  VT : ini=%.2f  min=%.2f  max=%.2f  final=%.4f  std10=%.4f  t_2%%=%.1fs  target=%.4f\n', ...
            s_vt.ini, s_vt.min, s_vt.max, s_vt.final, s_vt.std10, s_vt.ts, s_vt.target);
    end

    results{k} = struct('nome',cen{k,1},'alt',s_alt,'vt',s_vt, ...
                        'time_alt',sig_alt.Time,'data_alt',squeeze(sig_alt.Data), ...
                        'time_vt' ,sig_vt.Time ,'data_vt' ,squeeze(sig_vt.Data));
end

fig = figure('Name','HIL v1 - manobras','Position',[100 100 1200 700]);
for k = 1:n
    r = results{k};
    subplot(n,2,2*k-1);
    plot(r.time_alt, r.data_alt, 'LineWidth',1.2); grid on; hold on;
    yline(100,'r--','target');
    title(['[' r.nome ']  Altitude (m)']); xlabel('t (s)'); ylabel('Alt (m)');
    subplot(n,2,2*k);
    plot(r.time_vt , r.data_vt , 'LineWidth',1.2); grid on; hold on;
    yline(15.1117,'r--','target');
    title(['[' r.nome ']  VT (m/s)']); xlabel('t (s)'); ylabel('VT (m/s)');
end
sgtitle('HIL v1 - resposta a perturbacoes nas condicoes iniciais');

png_path = fullfile(repo,'Arduino','HIL_simulink','manobras_hil_v1.png');
exportgraphics(fig, png_path, 'Resolution',150);
fprintf('\nFigura salva em: %s\n', png_path);

clear hil_serial_step;
close_system(mdl,0);


function st = local_stats(sig, target)
    if isempty(sig), st = []; return; end
    t = sig.Time; d = squeeze(sig.Data);
    mask10 = t >= max(t)-10;
    err = abs(d - target);
    thr = 0.02 * abs(d(1) - target);
    if thr < 1e-6
        ts = NaN;
    else
        below = err < thr;
        idx   = find(~below, 1, 'last');
        if isempty(idx),       ts = 0;
        elseif idx == numel(t),ts = NaN;
        else,                  ts = t(idx+1);
        end
    end
    st = struct('ini',d(1),'min',min(d),'max',max(d),'final',d(end), ...
                'std10', std(d(mask10)), 'ts', ts, 'target', target);
end
