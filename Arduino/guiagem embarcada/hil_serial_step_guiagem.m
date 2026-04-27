function U = hil_serial_step_guiagem(sensors15)
% hil_serial_step_guiagem - Ponte serial PC<->Arduino para HIL com
% guiagem embarcada. Idem hil_serial_step.m, mas na 1a chamada faz
% upload da matriz WPs do base workspace via mensagem WP_UPLOAD.
%
% Protocolo:
%   PC -> MEGA  WP_UPLOAD : [0xBB 0x66] uint8 N + N*(4 floats LE)
%   MEGA -> PC  WP_ACK    : [0x66 0xBB] uint8 N
%   PC -> MEGA  SENSORS   : [0xAA 0x55] + 15 floats LE
%   MEGA -> PC  ACTUATORS : [0x55 0xAA] + 4 floats LE
%
% sensors15 inclui xN em s(13) e xE em s(14) (antes zerados).

persistent sp fail_count last_wp R_stop
if isempty(fail_count), fail_count = 0; end
MAX_FAILS = 5;

if isempty(sp) || ~isvalid(sp)
    try
        sp = serialport('COM3', 115200, 'Timeout', 1);
    catch ME
        error('hil_serial_step_guiagem:noPort', ...
            'Arduino nao encontrado em COM3 (%s).', ME.message);
    end
    pause(3);           % DTR reset
    flush(sp);
    fail_count = 0;

    % --- Upload de waypoints ---
    try
        WPs_local = evalin('base', 'WPs');
    catch
        error('hil_serial_step_guiagem:noWPs', ...
            'Variavel WPs nao encontrada no base workspace.');
    end
    N = size(WPs_local, 1);
    if N < 1 || N > 32
        error('hil_serial_step_guiagem:badN', ...
            'WPs deve ter entre 1 e 32 linhas (tem %d).', N);
    end
    write(sp, uint8([187 102]), 'uint8');     % 0xBB 0x66
    write(sp, uint8(N), 'uint8');
    write(sp, single(reshape(WPs_local(:,1:4)', 1, [])), 'single');
    ack = read(sp, 3, 'uint8');
    if numel(ack) ~= 3 || ack(1) ~= 102 || ack(2) ~= 187 || ack(3) ~= N
        try, delete(sp); catch, end
        sp = [];
        error('hil_serial_step_guiagem:wpAck', ...
            'Falha no ACK do upload de WPs (recebido: %s).', mat2str(ack));
    end
    fprintf('hil_serial_step_guiagem: %d waypoints enviados ao Mega.\n', N);
    last_wp = WPs_local(end, 1:2);   % [N, E] do ultimo WP
    R_stop  = 80;                     % mesmo R_accept do firmware
end

% --- Checa se chegou na zona do ultimo WP -> para a sim ---
xN_now = sensors15(13);
xE_now = sensors15(14);
if ~isempty(last_wp)
    dN = last_wp(1) - xN_now;
    dE = last_wp(2) - xE_now;
    if sqrt(dN*dN + dE*dE) <= R_stop
        fprintf('hil_serial_step_guiagem: ultimo WP alcancado, parando sim.\n');
        try, set_param(bdroot, 'SimulationCommand', 'stop'); catch, end
        U = [0.491387; 0.015456; 0; 0];
        return;
    end
end

ok = false;
try
    write(sp, uint8([170 85]), 'uint8');
    write(sp, single(sensors15(:)'), 'single');
    buf = read(sp, 18, 'uint8');
    if numel(buf) == 18 && buf(1) == 85 && buf(2) == 170
        U = double(typecast(uint8(buf(3:18)), 'single')');
        ok = true;
    end
catch
    ok = false;
end

if ~ok
    fail_count = fail_count + 1;
    warning('hil_serial_step_guiagem: sem resposta do Arduino (%d/%d)', fail_count, MAX_FAILS);
    if fail_count >= MAX_FAILS
        try, delete(sp); catch, end
        sp = [];
        error('hil_serial_step_guiagem:lostLink', ...
            'Arduino perdeu conexao (%d passos sem resposta). Abortando.', MAX_FAILS);
    end
    U = [0.491387; 0.015456; 0; 0];
else
    fail_count = 0;
end
end
