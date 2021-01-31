%%
%% Modbas - Frank Traenkle
%%
%% Parameters for Simulink model pt1_blockdiagram.slx
%%

clear variables; % clear workspace

P_T = 100e-6; % time constant [ s ]
P_k = 1; % gain [ 1 ]
P_x0 = 0; % initial value capacity voltage [ V ]

P_dt = 10e-6; % step size ODE solver [ s ]   