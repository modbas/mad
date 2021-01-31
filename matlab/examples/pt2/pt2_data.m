%%
%% Modbas - Frank Traenkle
%%
%% Parameters for Simulink model pt1_blockdiagram.slx
%%

clear variables; % clear workspace

P_T = 1e-3; % time constant of system with no damping [ s ]
P_D = 0.3; % damping constant [ 1 ]
P_k = 1; % gain [ 1 ]
P_x0 = 0; % initial position [ 1 ]
P_v0 = 0; % initial speed [ 1/s ]

% step size computation
l = roots([ P_T^2 , 2*P_D*P_T , 1 ]); % eigenvalues of PT2
disp([ 'P_dt must be less than ' ...
    num2str(0.1 * min(1 / abs(real(l(1))), 2*pi / abs(imag(l(1))))) ...
    ]);
P_dt = 1e-4; % chosen step size ODE solver [ s ]
