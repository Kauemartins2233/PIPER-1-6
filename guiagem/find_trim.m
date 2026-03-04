%% find_trim.m - Encontra o ponto de equilibrio (trim) para voo reto e nivelado
% Usa fminsearch para minimizar ||Xp|| variando [delta_T, delta_e, alpha].
%
% Condicoes de trim:
%   - Voo reto e nivelado (gamma = 0 -> theta = alpha)
%   - VT = 15 m/s
%   - beta = 0, phi = 0
%   - p = q = r = 0
%   - Altitude = 560m
%
% Uso: >> find_trim
%      (roda apos executar.m ter carregado par_aero, par_prop, par_gen)

fprintf('\n========== BUSCA DE TRIM ==========\n');

%% Verificar pre-requisitos
if ~exist('par_aero','var') || ~exist('par_prop','var') || ~exist('par_gen','var')
    error('Rode executar.m primeiro para carregar par_aero, par_prop, par_gen.');
end

%% Parametros fixos
VT_trim = 15;       % m/s
h_trim  = 560;      % m (altitude)

%% Chute inicial (valores do equilibrium.m)
alpha0  = 0.29129;
dT0     = 0.05;
dE0     = -1.17391;

x0 = [dT0, dE0, alpha0];

fprintf('  Chute inicial: dT=%.6f, dE=%.6f, alpha=%.6f rad (%.3f deg)\n', ...
    x0(1), x0(2), x0(3), rad2deg(x0(3)));

%% Funcao custo como funcao anonima
% Captura VT_trim, h_trim, par_gen, par_aero, par_prop por closure
trim_cost = @(xv) trim_cost_fn(xv, VT_trim, h_trim, par_gen, par_aero, par_prop);

%% Otimizacao
opts = optimset('TolFun', 1e-16, 'TolX', 1e-12, 'MaxFunEvals', 50000, ...
                'MaxIter', 10000, 'Display', 'off');

[x_opt, fval, exitflag] = fminsearch(trim_cost, x0, opts);

dT_trim_val = x_opt(1);
dE_trim_val = x_opt(2);
alpha_trim  = x_opt(3);
theta_trim  = alpha_trim;

fprintf('\n--- Resultado do Trim ---\n');
fprintf('  delta_T = %.8f  (era %.6f)\n', dT_trim_val, dT0);
fprintf('  delta_e = %.8f  (era %.6f)\n', dE_trim_val, dE0);
fprintf('  alpha   = %.8f rad = %.4f deg  (era %.6f rad = %.3f deg)\n', ...
    alpha_trim, rad2deg(alpha_trim), alpha0, rad2deg(alpha0));
fprintf('  exitflag = %d, custo residual = %.2e\n', exitflag, fval);

%% Montar Xe e Ue novos
u_trim = VT_trim * cos(alpha_trim);
v_trim = 0;
w_trim = VT_trim * sin(alpha_trim);

Xe_new = [u_trim; v_trim; w_trim; 0; 0; 0; 0; theta_trim; 0; 0; 0; -h_trim];
Ue_new = [dT_trim_val; dE_trim_val; 0; 0];

%% Verificar
Xp_old = dyn_rigidbody(0, Xe, Ue, par_gen, par_aero, par_prop);
Xp_new = dyn_rigidbody(0, Xe_new, Ue_new, par_gen, par_aero, par_prop);
fprintf('\n--- Verificacao ---\n');
fprintf('  ||Xp_old|| = %.6e  (trim antigo)\n', norm(Xp_old));
fprintf('  ||Xp_new|| = %.6e  (trim novo)\n', norm(Xp_new));
fprintf('  Derivadas: up=%.4e  vp=%.4e  wp=%.4e\n', Xp_new(1), Xp_new(2), Xp_new(3));
fprintf('  Rotacao:   pp=%.4e  qp=%.4e  rp=%.4e\n', Xp_new(4), Xp_new(5), Xp_new(6));
fprintf('  Cinemat:   phip=%.4e  thetap=%.4e  psip=%.4e\n', Xp_new(7), Xp_new(8), Xp_new(9));
fprintf('  Posicao:   xNp=%.4f  xEp=%.4f  xDp=%.6f\n', Xp_new(10), Xp_new(11), Xp_new(12));

%% Atualizar workspace
% Excluir xNp(10) e xEp(11) do criterio - sao velocidades de translacao
% esperadas em voo reto (xNp ~ VT). Verificar apenas derivadas do corpo
% (1-9) e taxa vertical xDp(12).
trim_quality = norm([Xp_new(1:9); Xp_new(12)]);
fprintf('  trim_quality (sem xNp,xEp) = %.6e\n', trim_quality);
if trim_quality < 1.0
    Xe = Xe_new;
    Ue = Ue_new;
    TrimInput = Ue';
    Xe_init = Xe;
    h_eq = -Xe(12);

    % Recalcular WPs com nova altitude
    WPs = [
         0,    0, h_eq,      15;
       300,    0, h_eq,      15;
       300,  200, h_eq,      15;
         0,  200, h_eq,      15;
         0,    0, h_eq,      15;
    ];

    fprintf('\n  >>> Xe, Ue, TrimInput, WPs ATUALIZADOS no workspace <<<\n');
    fprintf('  VT = %.2f m/s, alpha = %.4f deg, h = %.0f m\n', ...
        VT_trim, rad2deg(alpha_trim), h_eq);
    fprintf('  TrimInput = [%.6f, %.6f, %.6f, %.6f]\n', TrimInput);
    fprintf('\n  Pode simular diretamente (Ctrl+T).\n');
else
    fprintf('\n  AVISO: Trim ainda nao converge bem (trim_quality=%.2e).\n', trim_quality);
    fprintf('  Tente variar o chute inicial ou verificar o modelo aerodinamico.\n');
    fprintf('  Para aceitar mesmo assim:\n');
    fprintf('    Xe = Xe_new; Ue = Ue_new; TrimInput = Ue''; Xe_init = Xe;\n');
end

fprintf('\n========== FIM DO TRIM ==========\n');
