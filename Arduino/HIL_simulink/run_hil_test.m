% run_hil_test  -  Script de teste do HIL v1.
% Roda modeloNL1_HIL.slx e reporta estatisticas de Alt e VT nos ultimos 10 s.

cd(fileparts(fileparts(fileparts(mfilename('fullpath')))));
clear hil_serial_step;

fprintf('--- inicializar ---\n');
inicializar;

repo     = pwd;   % inicializar nao muda pwd; cd ja foi feito antes
mdl      = 'modeloNL1_HIL';
mdl_path = fullfile(repo,'Arduino','HIL_simulink',[mdl '.slx']);
fprintf('repo = %s\n', repo);
fprintf('--- load_system %s ---\n', mdl_path);
load_system(mdl_path);

set_param(mdl,'SignalLogging','on','SignalLoggingName','logsout', ...
    'StopTime','30','SaveOutput','on','SaveFormat','Dataset', ...
    'ReturnWorkspaceOutputs','on');

fprintf('--- sim (StopTime=30) ---\n');
t0  = tic;
out = sim(mdl);
fprintf('sim concluida em %.1f s wall-clock\n', toc(t0));

names = {};
sig_alt = [];  sig_vt = [];
if isprop(out,'logsout') && ~isempty(out.logsout)
    ls = out.logsout;
    fprintf('\nlogsout contem %d sinais:\n', ls.numElements);
    for i = 1:ls.numElements
        el = ls.getElement(i);
        names{end+1} = el.Name; %#ok<SAGROW>
        sz = size(el.Values.Data);
        fprintf('  [%d] %-30s size=%s\n', i, el.Name, mat2str(sz));
        nm = lower(el.Name);
        if isempty(sig_alt) && (contains(nm,'alt') || contains(nm,'h_'))
            sig_alt = el.Values;
        elseif isempty(sig_vt) && (contains(nm,'vt') || contains(nm,'vel') || contains(nm,'airspeed'))
            sig_vt = el.Values;
        end
    end
else
    fprintf('Nenhum logsout disponivel.\n');
end

fprintf('\n=== Resultado ===\n');
if ~isempty(sig_alt)
    t = sig_alt.Time;  d = squeeze(sig_alt.Data);
    mask = t >= max(t)-10;
    fprintf('  Alt: ini=%.2f  min=%.2f  max=%.2f  final=%.4f  std(ult10s)=%.4f  target=100\n', ...
        d(1), min(d), max(d), d(end), std(d(mask)));
else
    fprintf('  Alt: nao encontrado em logsout\n');
end
if ~isempty(sig_vt)
    t = sig_vt.Time;  d = squeeze(sig_vt.Data);
    mask = t >= max(t)-10;
    fprintf('  VT : ini=%.2f  min=%.2f  max=%.2f  final=%.4f  std(ult10s)=%.4f  target=15.11\n', ...
        d(1), min(d), max(d), d(end), std(d(mask)));
else
    fprintf('  VT : nao encontrado em logsout\n');
end

clear hil_serial_step;
close_system(mdl,0);
