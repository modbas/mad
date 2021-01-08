%%
%% Modbas- Frank Traenkle - $Id: laplacetable.m 19 2013-08-20 11:06:43Z ftraenkle $
%%
%% Parameter fuer Simulink-Modell pt1_blockdiagram.slx
%%

P_R = 100; % Ohmscher Widerstand [ Ohm ];
P_C = 1e-6; % Kapazitaet [ F ]

P_T = P_R * P_C; % Zeitkonstante PT1 [ s ]
P_k = 1; % Verstaerkungsfaktor PT1 [ V / V ]
P_x0 = 0; % Anfangswert Spannung an Kapazitaet [ V ]

P_u_step = 12; % Versorgungsspannung [ V ]
P_u_steptime = 1e-3; % Einschaltzeitpunkt [ s ]

P_dt = 1e-5; % Step size ODE solver [ s ]   