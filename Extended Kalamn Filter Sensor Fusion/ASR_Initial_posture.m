clear all

ASR_Parameters

% Initial Values
xCr_init = 0;
yCr_init = 0;
phi_init = deg2rad(65);
psi_init = deg2rad(5); 

Q_E      = [1 0 0;0 1 0;0  0 1];
P_init_E = [1 0 0;0 1 0;0  0 1];
R_E      = [1 0 0;0 1 0;0  0 1];
 
x_init = [xCr_init;yCr_init;phi_init];
dt = Ts;

Q_IMU      = [1 0 0;0 1 0;0  0 1];
P_init_IMU = [1 0 0;0 1 0;0  0 1];
R_IMU      = [1 0 0;0 1 0;0  0 1];