function U = hil_serial_step(sensors15)
% hil_serial_step - Chamada a cada passo pelo MATLAB Function block do
% modelo Simulink HIL. Envia 15 sensores (single) ao Mega, recebe 4
% atuadores (single) e devolve como vetor double 4x1.
%
% Protocolo (casa com arduino_controlador_manual.ino):
%   PC -> MEGA : [0xAA 0x55] + 15 floats LE (60 B)
%   MEGA -> PC : [0x55 0xAA] + 4  floats LE (16 B)
%
% Mantem o serialport aberto via variavel persistente. Fecha/reabre e
% descarta buffer na primeira chamada de cada simulacao.

persistent sp fail_count
if isempty(fail_count), fail_count = 0; end
MAX_FAILS = 5;   % passos consecutivos sem resposta antes de abortar

if isempty(sp) || ~isvalid(sp)
    try
        sp = serialport('COM3', 115200, 'Timeout', 1);
    catch ME
        error('hil_serial_step:noPort', ...
            'Arduino nao encontrado em COM3 (%s). Simulacao abortada.', ME.message);
    end
    pause(3);           % DTR reset do Mega
    flush(sp);
    fail_count = 0;
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
    warning('hil_serial_step: sem resposta do Arduino (%d/%d)', fail_count, MAX_FAILS);
    if fail_count >= MAX_FAILS
        try, delete(sp); catch, end
        sp = [];
        error('hil_serial_step:lostLink', ...
            'Arduino perdeu conexao (%d passos consecutivos sem resposta). Simulacao abortada.', MAX_FAILS);
    end
    U = [0.491387; 0.015456; 0; 0];
else
    fail_count = 0;
end
end
