%% setup_pid_blocks.m - Atualiza blocos PID do NL_guidance para PID completo
% Muda D e N de valores fixos (0 e 100) para variaveis do workspace.
% Rodar APOS executar.m e APOS abrir NL_guidance.slx.
%
% Uso:
%   executar          % carrega ganhos no workspace
%   open('NL_guidance.slx')
%   setup_pid_blocks  % atualiza os blocos PID
%
% O que este script faz:
%   1. Aplica termos D e N do workspace em todos os PIDs
%      (os blocos originais tinham D=0, N=100 hardcoded)
%   2. Muda anti-windup para back-calculation nos PIDs com saturacao
%   3. Configura solver ode15s com tolerancias apropriadas

model = 'NL_guidance';

% Verifica se o modelo esta aberto
if ~bdIsLoaded(model)
    error('Abra o modelo %s antes de rodar este script.', model);
end

%% ========== Verificar variaveis no workspace ==========
required_vars = {'C_alt', 'C_theta', 'C_vel', 'C_phi'};
for i = 1:length(required_vars)
    if ~evalin('base', sprintf('exist(''%s'',''var'')', required_vars{i}))
        error('Variavel %s nao encontrada no workspace. Rode executar.m primeiro.', required_vars{i});
    end
end

%% ========== Aplicar ganhos D e N nos blocos PID ==========
fprintf('\n--- Atualizando blocos PID ---\n');

pids = {
    % Block path                                                           D              N          Nome
    [model '/Autopilot1/Altitude Hold/PID Controller'],                   'C_alt.Kd',   'C_alt.N',   'Altitude';
    [model '/Autopilot1/Longitudinal Atitude Control/PID Controller'],    'C_theta.Kd', 'C_theta.N', 'Theta';
    [model '/Autopilot1/Velocity Control/PID Controller'],                'C_vel.Kd',   'C_vel.N',   'Velocidade';
    [model '/Autopilot1/Latero AutoPilotl/PID Controller'],               'C_phi.Kd',   'C_phi.N',   'Roll';
};

for i = 1:size(pids, 1)
    blk  = pids{i, 1};
    d_val = pids{i, 2};
    n_val = pids{i, 3};
    nome  = pids{i, 4};

    try
        % Verificar valores antes
        d_old = get_param(blk, 'D');
        n_old = get_param(blk, 'N');

        set_param(blk, 'D', d_val);
        set_param(blk, 'N', n_val);
        fprintf('  [%s] D: %s -> %s,  N: %s -> %s\n', nome, d_old, d_val, n_old, n_val);
    catch ME
        fprintf('  ERRO em %s: %s\n', nome, ME.message);
    end
end

%% ========== Anti-windup ==========
% Apenas para PIDs COM saturacao (Altitude Hold e Roll)
fprintf('\n--- Anti-windup ---\n');

pids_with_sat = {
    [model '/Autopilot1/Altitude Hold/PID Controller'],    'Altitude';
    [model '/Autopilot1/Latero AutoPilotl/PID Controller'], 'Roll';
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
fprintf('\n--- Blocos PID atualizados para PID completo ---\n');
fprintf('  Altitude:  Kp=%.4f, Ki=%.4f, Kd=%.4f, N=%.2f  [sat -0.17..0.26]\n', ...
    C_alt.Kp, C_alt.Ki, C_alt.Kd, C_alt.N);
fprintf('  Theta:     Kp=%.4f, Ki=%.4f, Kd=%.4f, N=%.2f  [sem sat]\n', ...
    C_theta.Kp, C_theta.Ki, C_theta.Kd, C_theta.N);
fprintf('  Velocidade:Kp=%.4f, Ki=%.4f, Kd=%.4f, N=%.2f  [sem sat]\n', ...
    C_vel.Kp, C_vel.Ki, C_vel.Kd, C_vel.N);
fprintf('  Roll:      Kp=%.4f, Ki=%.4f, Kd=%.4f, N=%.2f  [sat -0.43..0.43]\n', ...
    C_phi.Kp, C_phi.Ki, C_phi.Kd, C_phi.N);
fprintf('\nSalve o modelo (Ctrl+S) para manter as alteracoes.\n');
