%% Cálculo para a função equilibrium.m

%VT velocidade utilizada no AVL, além de ser aproximadamente a moda
% do log de voo
VT = 15; 
Alpha = 0.29129; %obtido do AVL
Beta = 0; %obtido do AVL
Theta = Alpha; %considerando ângulo de trajetória = 0

U = VT * cos(Alpha)*cos(Beta);
V = VT*sin(Beta);
W = VT*sin(Alpha)*cos(Beta);

%x = [u v w p q r phi theta psi pN pE h]'
%% valor de equilíbrio para o caso do log de voo
% altitude do log, moda aproximada em 560m
Xe = [U;V;W;0;0;0;0;Theta;0;0;0;-560];
%% valor de equilíbrio para o caso do Xplane
% Xe = [U;V;W;0;0;0;0;Theta;0;0;0;-1682.46594];
%x = [delta_T delta_e delta_a delta_r]'
%delta_T está em porcentagem, o valor do Marcelo era 0.0416 
% figura A.12 pg 140 da dissertação
Ue = [0.05; -1.17391; 0; 0];