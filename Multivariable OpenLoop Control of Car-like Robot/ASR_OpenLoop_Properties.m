% *************************************************************************
% Ackermann Steering Robot Model Properties
% Units = Meters
% NOTE: u1 = v (driving wheel velocity) & u2 = psi_dot (steering rate)
% *************************************************************************
clear all
close all
% Input Multiport Switch
%   Select Input Type as follows:
%       1. Sinusoidal
%       2. Piecewise Constant
%       3. Polynomial 
Input_Selector = 3;  

% Setup Parameters
T_Step  = 1e-3;         %!< Simulation Time Step
T_Total = 12;           %!< Total Simulation Time
N_Int   = 3;            %!< Number of Time Intervals
T_Int   = 12/N_Int;     %!< Time of One Interval
w       = 2*pi/T_Int;   %!< Omega (Sinusoidal Inputs)

% Robot Geometry
a = 1;                  %!< Distance Between Axles

% Initial Posture
x_init   = 4;           %!< Initial X Position
y_init   = 1;           %!< Initial Y Position
phi_init = pi/4;        %!< Initial Heading Orientation    
psi_init = 0;           %!< Initial Steering Orientation

% Target Posture
x_final   = 0;
y_final   = 0;
phi_final = 0;

%% Sinusoidal Inputs
% Interval 1
alpha_1 = -pi/2;
beta_1  = 0;
% Interval 2 (k = 1)
alpha_2 = -1;
beta_2  = -pi/(4*alpha_2);
% Interval 1 (k = 2)
alpha_3 = 2;
beta_3  = (3.6365*pi^2) / (2*alpha_3^2);
%% Piecewise-Constant Inputs
% Interval 1
PC_u1_1 = (x_final - x_init) / T_Total;
PC_u2_1 = 0.2344;
% Interval 2
PC_u1_2 = PC_u1_1;
PC_u2_2 = -0.2812;
% Interval 3
PC_u1_3 = PC_u1_1;
PC_u2_3 = 0.0469;
%% Polynomial Inputs
P_u1 = x_final - x_init / T_Total;
%Linear Equation Solver
A = [T_Total T_Total^2/2 T_Total^3/3;P_u1*(T_Total^2)/2 P_u1*(T_Total^3)/6 P_u1*(T_Total^4)/12; ...
    (P_u1^2)*(T_Total^3)/6 (P_u1^2)*(T_Total^4)/24 (P_u1^2)*(T_Total^5)/60];
c = inv(A)*[0;-1;-P_u1*T_Total-1];
c0 = c(1);
c1 = c(2);
c2 = c(3);