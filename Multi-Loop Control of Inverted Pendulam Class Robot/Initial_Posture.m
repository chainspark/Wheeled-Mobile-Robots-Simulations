clear all
close all

Parameters

% Initial Values
theta13d_init = 0;
theta23d_init = 0;
w3l_init = 0;

theta13_init = 0;
theta23_init = 0;

alpha_init = 0;
phi_init   = 0;

xCo_init = 0.5;
yCo_init = 0.5;
zCo_init = rw;

% Robot Posture computed using (xCo,yCo,phi and alpha)
u3_init = [cos(phi_init)*cos(alpha_init);sin(phi_init)*cos(alpha_init);-sin(alpha_init)];
l_init = [-sin(phi_init);cos(phi_init);0];
v3_init = [cos(phi_init)*sin(alpha_init);sin(phi_init)*sin(alpha_init);cos(alpha_init)];
Q3_init = [u3_init l_init v3_init];

co_init = [xCo_init;yCo_init;zCo_init];
c1_init = co_init - b/2*l_init;
c2_init = co_init + b/2*l_init;
c3_init = co_init - d*v3_init;

xC1_init = c1_init(1,1);
yC1_init = c1_init(2,1);
zC1_init = rw;

xC2_init = c2_init(1,1);
yC2_init = c2_init(2,1);
zC2_init = rw;

xC3_init = c3_init(1,1);
yC3_init = c3_init(2,1);
zC3_init = c3_init(3,1);

% Retrieving the Euler-Rodrigues parameters
if(trace(Q3_init)~=-1)
    q0_init = (trace(Q3_init)-1)/2;
    q_init = 1/2*[Q3_init(3,2)-Q3_init(2,3);Q3_init(1,3)-Q3_init(3,1);Q3_init(2,1)-Q3_init(1,2)];
    r0_init = sqrt((1+q0_init)/2);
    r_init = q_init/2/r0_init;
    rx_init = r_init(1,1);
    ry_init = r_init(2,1);
    rz_init = r_init(3,1);
    R_init = [0 -rz_init ry_init;rz_init 0 -rx_init;-ry_init rx_init 0];
end
if(trace(Q3_init)==-1)
    input('We must use the natural invariant in order to compute the ERP r & r0')
    r0_init = 0;
    eeT = 0.5*(Q3_init+eye(3))
    ey = sqrt(eeT(2,2)); ez = sqrt(eeT(3,3));
    r_init = [0;ey;-ez];
    rx_init = r_init(1);
    ry_init = r_init(2);
    rz_init = r_init(3);
end