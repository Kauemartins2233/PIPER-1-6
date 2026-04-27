function criar_modelo_HIL_guiagem()
% criar_modelo_HIL_guiagem - Gera modelo_HIL_guiagem.slx a partir de
% modeloNL1.slx, substituindo o subsystem `controle` por um MATLAB
% Function block que faz I/O serial com o Arduino Mega rodando
% arduino_guiagem_controle.ino.
%
% Diferenca em relacao a criar_modeloNL_HIL.m:
%   - O chart hil_io recebe DOIS inputs: sensors (15) do Mux original
%     E pos (2) tapado da sfunction_piper outputs [10,11] = [xN, xE].
%     O chart sobrescreve sensors(13:14) com pos antes de mandar pra
%     hil_serial_step_guiagem.
%
% Pre-req: inicializar.m rodado (Xe, Ue, par_* no base workspace).

src = which('modeloNL1.slx');
if isempty(src)
    error('modeloNL1.slx nao encontrado no path. Rode inicializar.m primeiro.');
end
dst_dir = fileparts(fileparts(mfilename('fullpath')));
dst = fullfile(dst_dir, 'modelo_HIL_guiagem.slx');

if bdIsLoaded('modelo_HIL_guiagem'), close_system('modelo_HIL_guiagem', 0); end
if exist(dst, 'file'), delete(dst); end
copyfile(src, dst);

mdl = 'modelo_HIL_guiagem';
load_system(dst);

% --- 1) Localiza controle e suas conexoes ---
ctrl = [mdl '/controle'];
ph = get_param(ctrl, 'PortHandles');
n_in  = numel(ph.Inport);
n_out = numel(ph.Outport);
fprintf('controle: %d inports, %d outports\n', n_in, n_out);

in_src = nan(n_in, 1);
for i = 1:n_in
    l = get_param(ph.Inport(i), 'Line');
    if l < 0, continue; end
    in_src(i) = get_param(l, 'SrcPortHandle');
end

out_dst_per_port = cell(n_out, 1);
for i = 1:n_out
    l = get_param(ph.Outport(i), 'Line');
    if l < 0
        out_dst_per_port{i} = [];
    else
        dd = get_param(l, 'DstPortHandle');
        out_dst_per_port{i} = dd(:);
    end
end

pos_ctrl = get_param(ctrl, 'Position');

% --- 1b) Localiza sfunction_piper para tapar xN, xE ---
sfblks = find_system(mdl, 'LookUnderMasks','all','FollowLinks','on', ...
                     'BlockType','S-Function');
sf_path = '';
for k = 1:numel(sfblks)
    fn = get_param(sfblks{k}, 'FunctionName');
    if strcmp(fn, 'sfunction_piper')
        sf_path = sfblks{k};
        break;
    end
end
if isempty(sf_path)
    error('sfunction_piper nao encontrado em %s', mdl);
end
fprintf('sfunction_piper em: %s\n', sf_path);
sf_ph = get_param(sf_path, 'PortHandles');
sf_outport = sf_ph.Outport(1);

% --- 2) Apaga o subsystem controle ---
delete_block(ctrl);

% --- 3) Cria MATLAB Function block ---
mfblk = [mdl '/hil_io'];
add_block('simulink/User-Defined Functions/MATLAB Function', mfblk, ...
          'Position', pos_ctrl);

script = [ ...
'function [U1, U2, U3, U4] = hil_io(sensors, pos)' newline ...
'%#codegen' newline ...
'U1 = 0; U2 = 0; U3 = 0; U4 = 0;' newline ...
'coder.extrinsic(''hil_serial_step_guiagem'');' newline ...
's = sensors(:);' newline ...
's(13) = pos(1);' newline ...
's(14) = pos(2);' newline ...
'tmp = zeros(4,1);' newline ...
'tmp = hil_serial_step_guiagem(s);' newline ...
'U1 = tmp(1);' newline ...
'U2 = tmp(2);' newline ...
'U3 = tmp(3);' newline ...
'U4 = tmp(4);' newline ...
'end' newline];

rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','Path',mfblk);
chart.Script = script;

inputs = chart.find('-isa','Stateflow.Data','-and','Scope','Input');
for k = 1:numel(inputs)
    if strcmp(inputs(k).Name,'sensors')
        inputs(k).Props.Array.Size = '15';
        inputs(k).DataType = 'double';
    elseif strcmp(inputs(k).Name,'pos')
        inputs(k).Props.Array.Size = '2';
        inputs(k).DataType = 'double';
    end
end

% --- 4) Mux para o input `sensors` ---
muxblk = [mdl '/hil_mux'];
add_block('simulink/Signal Routing/Mux', muxblk, ...
          'Inputs', num2str(n_in), ...
          'Position', [pos_ctrl(1)-60 pos_ctrl(2) pos_ctrl(1)-30 pos_ctrl(4)]);

% Selector para extrair xN, xE da sfunction_piper (saidas 10 e 11)
selblk = [mdl '/hil_pos_sel'];
add_block('simulink/Signal Routing/Selector', selblk, ...
          'IndexOptions', 'Index vector (dialog)', ...
          'Indices', '[10 11]', ...
          'InputPortWidth', '21', ...
          'Position', [pos_ctrl(1)-60 pos_ctrl(2)+80 pos_ctrl(1)-30 pos_ctrl(2)+100]);

try, set_param(mdl, 'SimulationCommand', 'update'); catch, end
ph2 = get_param(mfblk, 'PortHandles');
phm = get_param(muxblk, 'PortHandles');
phs = get_param(selblk, 'PortHandles');
fprintf('  MF block ports: in=%d out=%d\n', numel(ph2.Inport), numel(ph2.Outport));

% Conecta entradas originais do controle no Mux
for i = 1:n_in
    if isnan(in_src(i))
        cblk = [mdl '/hil_zero' num2str(i)];
        add_block('simulink/Sources/Constant', cblk, ...
                  'Value','[0;0;0]', ...
                  'Position',[pos_ctrl(1)-120 pos_ctrl(2)+30*(i-1) pos_ctrl(1)-95 pos_ctrl(2)+20+30*(i-1)]);
        phc = get_param(cblk,'PortHandles');
        add_line(mdl, phc.Outport(1), phm.Inport(i), 'autorouting','on');
    else
        add_line(mdl, in_src(i), phm.Inport(i), 'autorouting','on');
    end
end

% Mux -> chart input 1 (sensors)
el = get_param(ph2.Inport(1),'Line');
if el >= 0, try delete_line(el); catch, end; end
add_line(mdl, phm.Outport(1), ph2.Inport(1), 'autorouting','on');

% sfunction_piper -> Selector -> chart input 2 (pos)
add_line(mdl, sf_outport, phs.Inport(1), 'autorouting','on');
el = get_param(ph2.Inport(2),'Line');
if el >= 0, try delete_line(el); catch, end; end
add_line(mdl, phs.Outport(1), ph2.Inport(2), 'autorouting','on');

% Outputs do chart -> destinos antigos do controle
for i = 1:n_out
    if i > numel(ph2.Outport)
        warning('MF block soh tem %d outports; pulando outport %d', ...
                numel(ph2.Outport), i);
        continue;
    end
    dsts = out_dst_per_port{i};
    for j = 1:numel(dsts)
        el = get_param(dsts(j), 'Line');
        if el >= 0, try delete_line(el); catch, end; end
        add_line(mdl, ph2.Outport(i), dsts(j), 'autorouting','on');
    end
end

% --- 4b) Tap POS [xN xE alt] -> To Workspace 'POS' (pra plot3d_voo) ---
pos_sel = [mdl '/pos_sel'];
add_block('simulink/Signal Routing/Selector', pos_sel, ...
          'IndexOptions','Index vector (dialog)', ...
          'Indices','[10 11 12]', ...
          'InputPortWidth','21', ...
          'Position',[pos_ctrl(3)+30 pos_ctrl(2) pos_ctrl(3)+90 pos_ctrl(2)+30]);
pos_tw = [mdl '/POS'];
add_block('simulink/Sinks/To Workspace', pos_tw, ...
          'VariableName','POS','SaveFormat','Timeseries', ...
          'Position',[pos_ctrl(3)+130 pos_ctrl(2) pos_ctrl(3)+200 pos_ctrl(2)+30]);
phps = get_param(pos_sel,'PortHandles');
phpw = get_param(pos_tw, 'PortHandles');
add_line(mdl, sf_outport, phps.Inport(1), 'autorouting','on');
add_line(mdl, phps.Outport(1), phpw.Inport(1), 'autorouting','on');

% --- 5) Solver / pacing ---
set_param(mdl, 'Solver','ode4', 'FixedStep','0.01', 'StopTime','200');
set_param(mdl, 'EnablePacing','on', 'PacingRate','1');

save_system(mdl);
close_system(mdl);
fprintf('\nGerado: %s\n', dst);
fprintf('Rode em seguida: addpath tools; fix_sample_time\n');
end
