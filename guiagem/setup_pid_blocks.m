%% setup_pid_blocks.m - Atualiza blocos PID do NL_guidance para PID completo
% Aplica TODOS os ganhos do workspace nos blocos PID do modelo.
% Rodar APOS executar.m e APOS abrir NL_guidance.slx.
%
% Uso:
%   executar          % carrega ganhos no workspace
%   open('NL_guidance.slx')
%   setup_pid_blocks  % atualiza os blocos PID
%
% O que este script faz:
%   1. Aplica termos D e N do workspace em todos os PIDs internos
%   2. Configura Heading PID (P=2.5, I=0) e Gain1 (0.3) para guiagem
%   3. Muda anti-windup para back-calculation nos PIDs com saturacao
%   4. Configura solver ode15s com tolerancias apropriadas

model = 'NL_guidance';

% Verifica se o modelo esta aberto
if ~bdIsLoaded(model)
    error('Abra o modelo %s antes de rodar este script.', model);
end

%% ========== Detectar nome do subsistema Autopilot ==========
% O modelo pode usar Autopilot1 ou Autopilot2
ap_name = '';
subs = find_system(model, 'SearchDepth', 1, 'BlockType', 'SubSystem');
for i = 1:length(subs)
    nm = get_param(subs{i}, 'Name');
    if contains(nm, 'Autopilot')
        ap_name = nm;
        break;
    end
end
if isempty(ap_name)
    error('Subsistema Autopilot nao encontrado no modelo.');
end
fprintf('\n--- Subsistema detectado: %s ---\n', ap_name);
ap = [model '/' ap_name];

%% ========== Verificar variaveis no workspace ==========
required_vars = {'C_alt', 'C_theta', 'C_vel', 'C_phi'};
for i = 1:length(required_vars)
    if ~evalin('base', sprintf('exist(''%s'',''var'')', required_vars{i}))
        error('Variavel %s nao encontrada no workspace. Rode executar.m primeiro.', required_vars{i});
    end
end

%% ========== Aplicar ganhos D e N nos blocos PID internos ==========
fprintf('\n--- Atualizando blocos PID internos ---\n');

pids = {
    % Block path                                              D              N          Nome
    [ap '/Altitude Hold/PID Controller'],                   'C_alt.Kd',   'C_alt.N',   'Altitude';
    [ap '/Longitudinal Atitude Control/PID Controller'],    'C_theta.Kd', 'C_theta.N', 'Theta';
    [ap '/Velocity Control/PID Controller'],                'C_vel.Kd',   'C_vel.N',   'Velocidade';
    [ap '/Latero AutoPilotl/PID Controller'],               'C_phi.Kd',   'C_phi.N',   'Roll';
};

for i = 1:size(pids, 1)
    blk  = pids{i, 1};
    d_val = pids{i, 2};
    n_val = pids{i, 3};
    nome  = pids{i, 4};

    try
        d_old = get_param(blk, 'D');
        n_old = get_param(blk, 'N');
        set_param(blk, 'D', d_val);
        set_param(blk, 'N', n_val);
        fprintf('  [%s] D: %s -> %s,  N: %s -> %s\n', nome, d_old, d_val, n_old, n_val);
    catch ME
        fprintf('  ERRO em %s: %s\n', nome, ME.message);
    end
end

%% ========== Heading Controller (Guiagem) ==========
% CRITICO: Heading PID deve ter I=0 para guiagem LOS por waypoints.
% A referencia de proa muda ~90 graus abruptamente nas transicoes de WP,
% e o integral causa windup/oscilacoes.
fprintf('\n--- Heading Controller (Guiagem) ---\n');

blk_head = [ap '/Latero AutoPilotl/PID Controller1'];
try
    old_P = get_param(blk_head, 'P');
    old_I = get_param(blk_head, 'I');
    set_param(blk_head, 'P', '2.5');
    set_param(blk_head, 'I', '0');
    set_param(blk_head, 'D', '0');
    fprintf('  Heading PID: P=%s->2.5, I=%s->0, D=0\n', old_P, old_I);
catch ME
    fprintf('  ERRO Heading PID: %s\n', ME.message);
end

% Gain1: heading erro -> phi_ref (reduzido para guiagem)
blk_gain1 = [ap '/Latero AutoPilotl/Gain1'];
try
    old_g = get_param(blk_gain1, 'Gain');
    set_param(blk_gain1, 'Gain', '0.3');
    fprintf('  Gain1: %s -> 0.3\n', old_g);
catch ME
    fprintf('  ERRO Gain1: %s\n', ME.message);
end

% Saturation heading
blk_sat_head = [ap '/Latero AutoPilotl/Saturation'];
try
    old_lo = get_param(blk_sat_head, 'LowerLimit');
    old_hi = get_param(blk_sat_head, 'UpperLimit');
    set_param(blk_sat_head, 'UpperLimit', '1.0');
    set_param(blk_sat_head, 'LowerLimit', '-1.0');
    fprintf('  Saturation: [%s,%s] -> [-1.0, 1.0]\n', old_lo, old_hi);
catch ME
    fprintf('  ERRO Saturation: %s\n', ME.message);
end

%% ========== Anti-windup ==========
fprintf('\n--- Anti-windup ---\n');

pids_with_sat = {
    [ap '/Altitude Hold/PID Controller'],    'Altitude';
    [ap '/Latero AutoPilotl/PID Controller'], 'Roll';
};

for i = 1:size(pids_with_sat, 1)
    blk  = pids_with_sat{i, 1};
    nome = pids_with_sat{i, 2};
    try
        old_aw = get_param(blk, 'AntiWindupMode');
        set_param(blk, 'AntiWindupMode', 'back-calculation');
        fprintf('  [%s] Anti-windup: %s -> back-calculation\n', nome, old_aw);
    catch ME
        fprintf('  ERRO anti-windup %s: %s\n', nome, ME.message);
    end
end

%% ========== Solver ==========
fprintf('\n--- Solver ---\n');
set_param(model, 'Solver', 'ode15s');
set_param(model, 'MaxStep', '0.05');
set_param(model, 'RelTol', '1e-3');
set_param(model, 'AbsTol', '1e-4');
set_param(model, 'MinStep', '1e-8');
fprintf('  Solver: ode15s, MaxStep=0.05, RelTol=1e-3, AbsTol=1e-4\n');

%% ========== Resumo ==========
fprintf('\n--- Blocos PID atualizados ---\n');
fprintf('  Altitude:    Kp=%.4f, Ki=%.4f, Kd=%.4f, N=%.2f  [sat -0.17..0.26]\n', ...
    C_alt.Kp, C_alt.Ki, C_alt.Kd, C_alt.N);
fprintf('  Theta:       Kp=%.4f, Ki=%.4f, Kd=%.4f, N=%.2f\n', ...
    C_theta.Kp, C_theta.Ki, C_theta.Kd, C_theta.N);
fprintf('  Velocidade:  Kp=%.4f, Ki=%.4f, Kd=%.4f, N=%.2f\n', ...
    C_vel.Kp, C_vel.Ki, C_vel.Kd, C_vel.N);
fprintf('  Roll:        Kp=%.4f, Ki=%.4f, Kd=%.4f, N=%.2f  [sat -0.43..0.43]\n', ...
    C_phi.Kp, C_phi.Ki, C_phi.Kd, C_phi.N);
fprintf('  Heading:     P=2.5, I=0, D=0, Gain1=0.3  [sat -1.0..1.0]\n');
fprintf('\nSalve o modelo (Ctrl+S) para manter as alteracoes.\n');
