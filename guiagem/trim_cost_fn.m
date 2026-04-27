function cost = trim_cost_fn(x_opt, VT, h, par_gen, par_aero, par_prop)
%TRIM_COST_FN Funcao custo para busca de trim via fminsearch.
%   cost = trim_cost_fn(x_opt, VT, h, par_gen, par_aero, par_prop)
%
%   x_opt = [delta_T, delta_e, alpha]

    dT    = x_opt(1);
    dE    = x_opt(2);
    alpha = x_opt(3);
    theta = alpha;   % gamma = 0 (voo nivelado)

    % Velocidades no corpo
    u = VT * cos(alpha);
    v = 0;
    w = VT * sin(alpha);

    % Estado
    X = [u; v; w; 0; 0; 0; 0; theta; 0; 0; 0; -h];

    % Controle
    U = [dT; dE; 0; 0];

    % Derivadas
    Xp = dyn_rigidbody(0, X, U, par_gen, par_aero, par_prop);

    % Custo: queremos up=0, wp=0, qp=0
    % Peso maior em qp (momento de pitch) pois eh o mais sensivel
    cost = Xp(1)^2 + Xp(3)^2 + 10*Xp(5)^2;
end
