%
% Mini-Auto-Drive
%
% Vehicle Dynamics Parameters
%
% Copyright (c) 2020 Frank Traenkle
% http://www.modbas.de
%
% This file is part of Mini-Auto-Drive.
%
% Mini-Auto-Drive is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% Mini-Auto-Drive is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with Mini-Auto-Drive.  If not, see <http://www.gnu.org/licenses/>.

global P_p_frontaxle_1 P_p_rearaxle_1 P_p_rear_1 P_p_c_1 P_p_c_2 P_p_delta_max; 

%% Maximum number of cars for ROS receive message processing
P_car_cnt = 4;

%% Geometry
P_p_rearaxle_1 = 49e-3; % distance between IMU and rear axle [ m ]
P_p_frontaxle_1 = 50e-3; % distance between IMU and front axle[ m ]
P_p_rear_1 = -34e-3; % position of rear end of car [ m ]
P_p_c_1 = 167e-3; % length of car [ m ]
P_p_c_2 = 83e-3; % width of car [ m ]

%% Vehicle Parameters
P_p_v_min = 0.15; % speed threshold to switch from dynamics model to
                  % kinematics model to avoid singularities [ m/s ]
P_p_m = 132e-3; % mass [ kg ]
P_p_J = 192e-6; % moment of inertia (yaw) [ kg*m^2 ]

%% Magic Formula Coefficient
P_p_Br = 0.0014;
P_p_Cr = 16.398;
P_p_Dr = 135.9653;
P_p_Bf = 0.025;
P_p_Cf = 114.3442;
P_p_Df = 0.8136;

%% Longitudinal Dynamics

% Steady-State Characteristic un -> speed
% umeas = [ -0.4 -0.3 -0.28 -0.25 0 0.2 0.23 0.25 0.3 0.35 0.4 0.45 0.5 0.55 0.6 ];
% vmeas = [ -0.7 -0.3  -0.2     0 0   0  0.2 0.35 0.6  0.9 1.1 1.35 1.5  1.6 1.7 ];
% u1 = -0.25;
% u2 = 0.23;
% v2 = 0.2;
% figure(1), clf;
% plot(umeas, vmeas, 'bx');
% grid on;
% hold on;
% plot(umeas(1:end-1), diff(vmeas), 'mx');
% k = 5;
% v = k * (umeas - u2) + u2;
% plot(umeas, v, 'rx');


% Leerlauf-Deadzone: -0.2 <= u <= 0.2

P_p_k = 2.51; % gain [ m/s ]
P_p_T = 316e-3; % time constant [ s ]
P_p_uTt = 60e-3; % input dead time [ s ] 
P_p_un_max = 1; % maximum motor input signal [ 1 ] 
P_p_un_min = -P_p_un_max; % minimum motor input signal [ 1 ]
P_p_un_slow_max = 0.328; % maximum motor input signal [ 1 ] 
P_p_un_slow_min = -P_p_un_max; % minimum motor input signal [ 1 ]
P_p_un_friction_pos = 0.2; % dead zone
P_p_un_friction_neg = -P_p_un_friction_pos; % dead zone

%% Lateral Dynamics
P_p_delta_max = 21.58/180*pi; % maximum steering angle [ rad ]
P_p_delta_min = -P_p_delta_max; % minimum steering angle [ rad ]
P_p_l = abs(P_p_frontaxle_1 + P_p_rearaxle_1); % wheel base [ m ]
P_p_lr = P_p_rearaxle_1;
P_p_lf = P_p_frontaxle_1;
P_p_delta_Tt = P_p_uTt; % servo dead time [ s ]

%% Computer vision
P_p_output_Tt = 40e-3; % image processing dead time [ s ]
P_p_sstd = 3e-3; % [ m ];
P_p_psistd = deg2rad(1); % [ rad ];
%P_p_vstd = 0.3; % [ m/s ];

%% Total dead time
P_p_Tt = P_p_uTt + P_p_output_Tt;

%% Initial Conditions
P_p_v0 = 0.0;   % speed [ m/s ]
P_p_s10 = 1.7;  % s1 position [ m ] Carid 0
P_p_s20 = 1.7;  % s2 position [ m ] Carid 0
P_p_s11 = 0.4;  % s1 position [ m ] Carid 1
P_p_s21 = 0.3;  % s2 position [ m ] Carid 1
P_p_s12 = 1.3;  % s1 position [ m ] Carid 2
P_p_s22 = 0.4;  % s2 position [ m ] Carid 2
P_p_psi0 = pi; % yaw angle [ rad ]
P_p_psi1 = 0; % yaw angle [ rad ]
P_p_psi2 = 0; % yaw angle [ rad ]
% Magic Tire Model (Section 5.5)
% vc1, s1, s2, psi, dpsi/dt, vc2, path length x
P_p_xvec0 = [ P_p_v0; P_p_s10; P_p_s20; P_p_psi0; 0; 0; 0 ];
P_p_xvec1 = [ P_p_v0; P_p_s11; P_p_s21; P_p_psi1; 0; 0; 0 ];
P_p_xvec2 = [ P_p_v0; P_p_s12; P_p_s22; P_p_psi2; 0; 0; 0 ];
% Magic Tire Model (Section 5.5)
% v, s1, s2, psi, dpsi/dt, beta, path length x
P_p_xkinvec0 = [ P_p_v0; P_p_s10; P_p_s20; P_p_psi0; 0 ];
P_p_xkinvec1 = [ P_p_v0; P_p_s11; P_p_s21; P_p_psi1; 0 ];
P_p_xkinvec2 = [ P_p_v0; P_p_s12; P_p_s22; P_p_psi2; 0 ];

%% Bus Objects
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 's1';
elems(2) = Simulink.BusElement;
elems(2).Name = 's2';
elems(3) = Simulink.BusElement;
elems(3).Name = 'psi';
elems(4) = Simulink.BusElement;
elems(4).Name = 'beta';
elems(5) = Simulink.BusElement;
elems(5).Name = 'v';
elems(6) = Simulink.BusElement;
elems(6).Name = 'x';
elems(7) = Simulink.BusElement;
elems(7).Name = 'time';
elems(7).DataType = 'uint8';
CAROUTPUTSEXT = Simulink.Bus;
CAROUTPUTSEXT.Elements = elems;

P_caroutputsext = Simulink.Bus.createMATLABStruct('CAROUTPUTSEXT', [], [1 P_car_cnt]);

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'cmd';
elems(1).DataType = 'uint8';
CarInputsCmdHalt = uint8(0);
CarInputsCmdForward = uint8(1);
CarInputsCmdReverse = uint8(2);
CarInputsCmdSlow = uint8(3);
CarInputsCmdCharge = uint8(4);
elems(2) = Simulink.BusElement;
elems(2).Name = 'pedals';
elems(3) = Simulink.BusElement;
elems(3).Name = 'steering';
CARINPUTS = Simulink.Bus;
CARINPUTS.Elements = elems;

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 's1';
elems(2) = Simulink.BusElement;
elems(2).Name = 's2';
elems(3) = Simulink.BusElement;
elems(3).Name = 'psi';
elems(4) = Simulink.BusElement;
elems(4).Name = 'v';
CTRLREFERENCE = Simulink.Bus;
CTRLREFERENCE.Elements = elems;

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 's1';
elems(2) = Simulink.BusElement;
elems(2).Name = 's2';
elems(3) = Simulink.BusElement;
elems(3).Name = 'psi';
CAROUTPUTS = Simulink.Bus;
CAROUTPUTS.Elements = elems;

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'vmax';
elems(2) = Simulink.BusElement;
elems(2).Name = 'type';
elems(2).DataType = 'uint8';
ManeuverTypeHalt = uint8(0);
ManeuverTypePark = uint8(1);
ManeuverTypeCharge = uint8(2);
ManeuverTypePathFollow = uint8(3);
elems(3) = Simulink.BusElement;
elems(3).Name = 'xManeuverEnd';
elems(4) = Simulink.BusElement;
elems(4).Name = 'lapCount';
elems(4).DataType = 'uint32';
elems(5) = Simulink.BusElement;
elems(5).Name = 'disableLaneMonitor';
elems(5).DataType = 'boolean';
elems(6) = Simulink.BusElement;
elems(6).Name = 'isnew';
elems(6).DataType = 'boolean';
MANEUVER = Simulink.Bus;
MANEUVER.Elements = elems;

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'state';
elems(1).DataType = 'uint8';
elems(2) = Simulink.BusElement;
elems(2).Name = 'x';
elems(3) = Simulink.BusElement;
elems(3).Name = 'segment';
elems(3).DataType = 'int32';
elems(4) = Simulink.BusElement;
elems(4).Name = 'v';
elems(5) = Simulink.BusElement;
elems(5).Name = 'batteryLow';
elems(5).DataType = 'boolean';
elems(6) = Simulink.BusElement;
elems(6).Name = 'timeout';
elems(6).DataType = 'boolean';
elems(7) = Simulink.BusElement;
elems(7).Name = 'lap';
elems(7).DataType = 'uint32';
elems(8) = Simulink.BusElement;
elems(8).Name = 'epsi';
elems(9) = Simulink.BusElement;
elems(9).Name = 'ex';
elems(10) = Simulink.BusElement;
elems(10).Name = 'ey';
MANEUVERSTATE = Simulink.Bus;
MANEUVERSTATE.Elements = elems;

P_max_breakslen = 256;
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'breakslen';
elems(1).DataType = 'uint32';
elems(2) = Simulink.BusElement;
elems(2).Name = 'points';
elems(2).Dimensions = [ 3 P_max_breakslen ];
elems(3) = Simulink.BusElement;
elems(3).Name = 'coefs';
elems(3).Dimensions = [ P_max_breakslen*2  4 ];
elems(4) = Simulink.BusElement;
elems(4).Name = 'segments';
elems(4).DataType = 'uint32';
elems(4).Dimensions = [ 1 P_max_breakslen ];
elems(5).Name = 'periodic';
elems(5).DataType = 'boolean';
SPLINE = Simulink.Bus;
SPLINE.Elements = elems;

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'batteryVoltage';
elems(2) = Simulink.BusElement;
elems(2).Name = 'batteryLow';
elems(2).DataType = 'boolean';
elems(3) = Simulink.BusElement;
elems(3).Name = 'err';
elems(3).DataType = 'uint8';
elems(4) = Simulink.BusElement;
elems(4).Name = 'calibFlag';
elems(4).DataType = 'boolean';
elems(5) = Simulink.BusElement;
elems(5).Name = 'state';
elems(5).DataType = 'uint8';
elems(6) = Simulink.BusElement;
elems(6).Name = 'motorCurrent';
elems(7) = Simulink.BusElement;
elems(7).Name = 'isnew';
elems(7).DataType = 'boolean';
CARSTATE = Simulink.Bus;
CARSTATE.Elements = elems;
