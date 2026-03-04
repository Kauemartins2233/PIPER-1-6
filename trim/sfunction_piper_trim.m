% -----------------------------------------------
% Function - Piper J Cub 1/4 Equations of Motion
% -----------------------------------------------
% Inputs:
%           1- t,x,u,flag,Xe: Model Initilization of s-function with Aircraft States at Equilibrium 
%               Xe(1)  = u_e
%               Xe(2)  = v_e
%               Xe(3)  = w_e
%               Xe(4)  = p_e
%               Xe(5)  = q_e
%               Xe(6)  = r_e
%               Xe(7)  = phi_e
%               Xe(8)  = theta_e
%               Xe(9)  = psi_e
%               Xe(10) = xN_e
%               Xe(11) = xE_e
%               Xe(12) = xD_e
% Output: 
%           case 0 (Initialization):
%               1- sys,x0,str,ts,simStateCompliance: Model Initialization with Aircraft States at Equilibrium
%           case 1 (Derivatives):
%               1- sys: State Derivatives
%                   sys(1) = up
%                   sys(2) = vp
%                   sys(3) = wp
%                   sys(4) = pp
%                   sys(5) = qp
%                   sys(6) = rp
%                   sys(7) = phip
%                   sys(8) = thetap
%                   sys(9) = psip
%                   sys(10) = xNp
%                   sys(11) = xEp
%                   sys(12) = xDp
%           case 3 (Output):
%               1- sys: Output Vector
%                   sys(1)  = VT
%                   sys(2)  = alpha
%                   sys(3)  = beta
%                   sys(4)  = p
%                   sys(5)  = q
%                   sys(6)  = r
%                   sys(7)  = phi
%                   sys(8)  = theta
%                   sys(9)  = psi
%                   sys(10) = xN
%                   sys(11) = xE
%                   sys(12) = -xD
%                   sys(13) = u
%                   sys(14) = v
%                   sys(15) = w
%                   sys(16) = up
%                   sys(17) = vp
%                   sys(18) = wp
%                   sys(19) = pp
%                   sys(20) = qq
%                   sys(21) = rp

function [sys,x0,str,ts,simStateCompliance]=sfunction_piper_trim(t,x,u,flag,par_gen,par_aero,par_prop,Xe) % Do not delete or change t,x,u,flag and sys,x0,str,ts
    switch flag
        %%%%%%%%%%%%%%%%%%
        % Initialization %
        %%%%%%%%%%%%%%%%%%
        case 0         
            [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(Xe);

        %%%%%%%%%%%%%%%
        % Derivatives %
        %%%%%%%%%%%%%%%
        case 1
            sys = dyn_rigidbody(t,x,u,par_gen,par_aero,par_prop);

        %%%%%%%%%%%%%%%%%%%%%%%%
        % Update and Terminate %
        %%%%%%%%%%%%%%%%%%%%%%%%
        case {2,9}
            sys = []; % do nothing

        %%%%%%%%%%
        % Output %
        %%%%%%%%%%
        case 3
             % Calcular outputs com protecao contra NaN/Inf/complex
             ub = x(1); vb = x(2); wb = x(3);
             VT = sqrt(ub^2 + vb^2 + wb^2);
             if VT < 1e-6, VT = 1e-6; end  % protege divisao por zero
             alpha = atan2(wb, ub);          % atan2 nunca da Inf
             beta  = asin(max(-1, min(1, vb/VT)));  % clamp para [-1,1]

             % Saidas observaveis (12x1)
             y = [VT; alpha; beta; x(4); x(5); x(6); x(7); x(8); x(9); x(10); x(11); -x(12)];

             % Derivadas (para monitoramento)
             Xp = dyn_rigidbody(t,x,u,par_gen,par_aero,par_prop);

             % Montar saida 21x1
             bv = [ub; vb; wb];        % 3x1 body velocities
             dv = Xp(1:6); dv = dv(:); % 6x1 derivadas

             sys = real([y; bv; dv]);   % 21x1

             % Proteger contra NaN/Inf
             if any(~isfinite(sys))
                 fprintf('SFUNC WARN t=%.6f: NaN/Inf detectado! x=%s u=%s\n', t, mat2str(x',4), mat2str(u',4));
                 sys(~isfinite(sys)) = 0;  % substituir por 0 para nao travar
             end

        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
end



% ---------------------------------------------------------------------------------------
% Function - Return the sizes, initial conditions, and sample times for the S-function.
% ---------------------------------------------------------------------------------------
% Inputs:
%           1- Xe: Aircraft States at Equilibrium
%               Xe(1)  = u_e
%               Xe(2)  = v_e
%               Xe(3)  = w_e
%               Xe(4)  = p_e
%               Xe(5)  = q_e
%               Xe(6)  = r_e
%               Xe(7)  = phi_e
%               Xe(8)  = theta_e
%               Xe(9)  = psi_e
%               Xe(10) = xN_e
%               Xe(11) = xE_e
%               Xe(12) = xD_e
% Output: 
%           1- sys,x0,str,ts,simStateCompliance: Model Initialization at the States inputted

function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(Xe)
    sizes = simsizes;
    sizes.NumContStates  = 12;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 21;
    sizes.NumInputs      = 4;
    sizes.DirFeedthrough = 0;  % evita loop algebrico; outputs 16-21 usam u do passo anterior (ok p/ monitoramento)
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);
    str = [];
    x0  = Xe;
    ts  = [0 0];   % sample time: [period, offset]

    % specify that the simState for this s-function is same as the default
    simStateCompliance = 'DefaultSimState';

end % mdlInitializeSizes