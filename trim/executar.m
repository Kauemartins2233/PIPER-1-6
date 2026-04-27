 clear vars;clc;

% Adiciona path das funcoes compartilhadas (dyn_rigidbody, aerodynamics, etc.)
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'modelos', 'Não Linear'));

% Modelo Latero-Direcional
load('Piper_bkp_AVL_1_6_v2.mat');


% [A, B, C, D] = decoupling(2, par_gen, par_aero, par_prop);
% states = {'\beta' 'p' 'r' '\phi' '\psi'};
% inputs = {'\delta_a' '\delta_r'};
% outputs = {'\beta' 'p' 'r' '\phi' '\psi'};
% aeronave = ss(A, B, C , D, 'statename',states,...
%                            'inputname',inputs,...
%                            'outputname',outputs);                       
% clearvars A B C D; 

% Carrega entradas e estados de equilibrio;
equilibrium;

disp('executado');

