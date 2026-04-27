%% Aircraft longitudinal cost function
% Author
% Date
% Version
% Last modification

%% Function
function J = pipertrimcostfunction(x,Xe)

Xe(3) = Xe(1)*tan(x(5));
Xe(8) = x(5);

%% Get sim data
SimOptions = simget('q2_trim_modelo24a');
% Make TrimInput visible to those Constant blocks
assignin('base','TrimInput', x(:).');  % row vector
% Assignin
assignin('base','Xe',Xe);

% Run Simulation
SimOutputs = sim('q2_trim_modelo24a',[0 0],SimOptions);



%% Get data from simulation output
u_dot = SimOutputs.Y.signals.values(16)
v_dot = SimOutputs.Y.signals.values(17)
w_dot = SimOutputs.Y.signals.values(18)
p_dot = SimOutputs.Y.signals.values(19)
q_dot = SimOutputs.Y.signals.values(20)
r_dot = SimOutputs.Y.signals.values(21)

% x_dot = [u_dot^2 w_dot^2 100*q_dot^2];
x_dot = [u_dot v_dot w_dot p_dot q_dot r_dot];

% Pesos
W = [1 1 1 1 10 1];

%% Calculate cost function
J = x_dot.*W*x_dot';

assignin('base','derivadas',[u_dot v_dot w_dot p_dot q_dot r_dot]);