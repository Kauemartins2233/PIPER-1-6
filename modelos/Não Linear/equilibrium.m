%% Cálculo para a função equilibrium.m
%
% Valores de trim obtidos via trimagem_piper.m (fmincon, interior-point).
% Resultado: fval = 1.95e-11, derivadas ~zero.
%
% Condição de voo: VT=15 m/s, h=100m, voo reto nivelado.
%
% Valores ANTIGOS (do AVL, NAO eram equilibrio real — ||Xp0|| = 201):
%   Alpha_old = 0.29129 rad (16.7 deg)
%   dT_old    = 0.05
%   dE_old    = -1.17391
%
% Valores do trim_vector.mat (trimagem_piper.m com fmincon):
%   Theta = -0.121684 rad (-6.97 deg)
%   dT    = 0.491387
%   dE    = 0.015456

VT = 15;                     % m/s (velocidade de trim)
Theta = -0.121684;           % rad (obtido via trimagem_piper.m / fmincon)
Beta = 0;
Alpha = Theta;               % gamma = 0 (voo nivelado)

U = VT;                      % aproximacao u ~ VT (consistente com trimagem_piper)
V = 0;
W = VT * tan(Theta);         % w = VT * tan(theta), conforme pipertrimcostfunction

%x = [u v w p q r phi theta psi pN pE h]'
%% valor de equilíbrio para voo reto nivelado a 100m
Xe = [U;V;W;0;0;0;0;Theta;0;0;0;-100];

%% valor de equilíbrio para o caso do Xplane
% Xe = [U;V;W;0;0;0;0;Theta;0;0;0;-1682.46594];

% x = [delta_T delta_e delta_a delta_r]'
Ue = [0.491387; 0.015456; 0; 0];