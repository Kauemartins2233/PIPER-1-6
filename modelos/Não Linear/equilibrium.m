%% Cálculo para a função equilibrium.m
%
% Valores de trim obtidos via find_trim.m (fminsearch minimizando derivadas
% do corpo). Resultado: trim_quality = 2.7e-12, exitflag = 1.
%
% Valores ANTIGOS (do AVL, NAO eram equilibrio real — ||Xp0|| = 201):
%   Alpha_old = 0.29129 rad (16.7 deg)
%   dT_old    = 0.05
%   dE_old    = -1.17391
%
% Valores NOVOS (equilibrio verdadeiro com dyn_rigidbody):
%   Alpha = -0.11692760 rad (-6.70 deg)
%   dT    = 0.47372918
%   dE    = 0.01635260

VT = 15;                     % m/s (velocidade de trim)
Alpha = -0.11692760;         % rad (obtido via find_trim.m)
Beta = 0;
Theta = Alpha;               % gamma = 0 (voo nivelado)

U = VT * cos(Alpha)*cos(Beta);
V = VT*sin(Beta);
W = VT*sin(Alpha)*cos(Beta);

%x = [u v w p q r phi theta psi pN pE h]'
%% valor de equilíbrio para voo reto nivelado a 560m
Xe = [U;V;W;0;0;0;0;Theta;0;0;0;-560];

%% valor de equilíbrio para o caso do Xplane
% Xe = [U;V;W;0;0;0;0;Theta;0;0;0;-1682.46594];

% x = [delta_T delta_e delta_a delta_r]'
Ue = [0.47372918; 0.01635260; 0; 0];