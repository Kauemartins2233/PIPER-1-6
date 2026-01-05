%% Trimagem do Piper
clear
clc

%% Load data
% Modelo Latero-Direcional
load('Piper_bkp_AVL_1_6_v2.mat');
% load('Sato_longitudinal_Piper_1_6_M1.mat');
equilibrium;

%% Trim states

% Flight Condition
% simu.param.V0 = 15; % m/s
simu.trim.velocities = [15 0 0]; % m/s
simu.trim.rates = [0 0 0]; % rad/s 
simu.trim.angles = [0 0.08 0]; % rad
simu.trim.altitude = 100; % m

Xe(1)  = simu.trim.velocities(1);
Xe(2)  = simu.trim.velocities(2);
Xe(3)  = simu.trim.velocities(3);
Xe(4)  = simu.trim.rates(1);
Xe(5)  = simu.trim.rates(2);
Xe(6)  = simu.trim.rates(3);
Xe(7)  = simu.trim.angles(1);
Xe(8)  = simu.trim.angles(2);
Xe(9)  = simu.trim.angles(3);
Xe(10) = 0;
Xe(11) = 0;
Xe(12) = -simu.trim.altitude;

%% Otimização via fmincon

% fmincon param
options.TolFun = 1e-9;
options.MaxFunctionsEvaluation = 1000;
options.MaxIterations = 1000;
options.TolCon = 1e-10;
options.Display = 'on';
options.TolX = 1e-10;
options.Algorithm = 'interior-point';

lb = [0.1 -30*pi/180 -5*pi/180 -5*pi/180 -30*pi/180]; % thr elev ail rudd theta
ub = [0.9 30*pi/180 5*pi/180 5*pi/180 30*pi/180];
TrimInput = (lb+ub)/2;

%% Fmincon
[INPUTS,fval] = fmincon(@(x) pipertrimcostfunction(x,Xe),TrimInput,...
    [],[],[],[],lb,ub,[],options);

INPUTS
derivadas

SimOptions = simget('q2_trim_modelo24a');
out = sim('q2_trim_modelo24a',[0 10],SimOptions);


figure(1)
subplot(2,1,1)
plot(out.tout,out.Y.signals.values(:,16:18))
title('Trim Evidence')
axis([0 10 -0.01 0.01])
legend('u_{dot}','v_{dot}','w_{dot}')
subplot(2,1,2)
plot(out.tout,out.Y.signals.values(:,19:21))
axis([0 10 -0.01 0.01])
legend('p_{dot}','q_{dot}','r_{dot}')

figure(2)
subplot(3,1,1)
plot(out.tout,out.Y.signals.values(:,1))
title('V_{T}')
legend('V_{T}')
axis([0 10 10 20])
subplot(3,1,2)
plot(out.tout,rad2deg(out.Y.signals.values(:,2)))
hold on
plot(out.tout,rad2deg(out.Y.signals.values(:,8)))
title('Angles - deg')
legend('\alpha','theta')
axis([0 10 -8 -6])
subplot(3,1,3)
plot(out.tout,out.Y.signals.values(:,12))
title('Altitude (m)')
legend('Altitude')
axis([0 10 90 110])
