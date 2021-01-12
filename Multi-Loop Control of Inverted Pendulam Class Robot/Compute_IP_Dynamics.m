function output = Compute_IP_Dynamics(tau1,tau2,theta13d,theta23d,w3l,cox,...
    coy,coz,rx,ry,rz,r0,theta13,theta23)

% Inputs of the function "compute_dynamics" :  
% tau1 & tau2 : torques applied to wheels 1 & 2, inputs of the system
% [theta13d ; theta23d ; w3l] : vector of independent velocities
% co : position vector of Co, the geometric centre of the central body
% r & r0 : Euler-Rodrigues parameters describing the orientation of the central body w.r.t F0
% theta13 & theta23 : angular displacements of wheels 1 & 2 w.r.t. the central body

Parameters

% Defining the state vector 
x1 = theta13; 
x2 = theta23;
x3 = cox; 
x4 = coy; 
x5 = coz;
x6 = rx; 
x7 = ry; 
x8 = rz; 
x9 = r0;
x10 = theta13d; 
x11 = theta23d; 
x12 = w3l;

% Computation of Q3 = [u3 l v3]
r = [x6;x7;x8];
R = [0 -x8 x7;x8 0 -x6;-x7 x6 0];
Q3 = (x9^2 - r'*r)*eye(3) + 2*r*r' + 2*x9*R;
u3 = Q3(:,1); l = Q3(:,2); v3 = Q3(:,3);

% Useful Substitutions
k = [0;0;1];
n = k;
h = cross(l,n);
v3xn = cross(v3,n);
hu3 = h'*u3;
hv3 = h'*v3;

hk = h'*k;
u3k = u3'*k;
v3xnk = v3xn'*k;

% Vector of independent velocities v
v = [x10;x11;x12];

% Time-rate of change of co, r & r0
w3p = (rw/b)*(x10-x11);
w3 = w3p*n + w3l*l;
cod = rw/2*(x10+x11+2*x12)*h;
rd = 1/2*(x9*eye(3)-R)*w3;
r0d = -1/2*r'*w3;

% Computation of vd
Ie = (rw/b)^2*( (Icu+mc*d^2)*hv3^2 + Icv*hu3^2 );
If = mc*(rw*d/2)*hu3;

Ja = 0.5/(Ia-Ib+2*Ie);
Jb = Ic/(Ic*Id-2*If^2);
Jc = (Ic-If)/(Ic*Id-2*If^2);
Jd = (Id/2-If)/(Ic*Id-2*If^2);

Ca = mc*(rw*d^2/b)*hu3*hv3;
Cb = mc*(rw^2*d/b)*hv3;
Cc = (rw/b)*(Icu-Icv)*hu3*hv3;
Cd = Ca + Cc;

Fa = Ja*(-4*Cd*w3p*w3l+Cb*w3p*(theta13d+theta23d+2*w3l));
Fb = Jb*(b/rw)*Cd*w3p^2 + (Jc-Jb)*(b/rw)*Cb*(w3p^2+w3l^2);
Fc = - Jc*(b/rw)*Cd*w3p^2 - Jd*(b/rw)*Cb*(w3p^2+w3l^2);

Ga = -2*Ja*mc*g*d*(rw/b)*v3xnk;
Gb = Jb*mc*g*d*u3k + (Jc-Jb)*(2*mw+mc)*g*rw*hk;
Gc = - Jc*mc*g*d*u3k - Jd*(2*mw+mc)*g*rw*hk;

theta13dd = Fa + Fc + Ga + Gc + (Ja+Jc+Jd)*tau1 + (-Ja+Jc+Jd)*tau2;
theta23dd = -Fa + Fc - Ga + Gc + (-Ja+Jc+Jd)*tau1 + (Ja+Jc+Jd)*tau2;
w3ld = Fb + Gb -Jc*tau1 - Jc*tau2;

% Extra outputs
co = [x3;x4;x5];
c1 = co - b/2*l;
c2 = co + b/2*l;
c3 = co - d*v3;

verif_C1 = c1(3,1) - rw;
verif_C2 = c2(3,1) - rw;
verif_Co = co(3,1) - rw;
verif_ERP2 = norm(r,2)^2 + r0^2 - 1;
verif_w3l = w3l - 2*(r0*l'*rd - r0d*l'*r + l'*(R*rd));

% Output vector
output(1,1) = c1(1,1);    % xC1
output(2,1) = c1(2,1);    % yC1
output(3,1) = c1(3,1);    % zC1
output(4,1) = c2(1,1);    % xC2
output(5,1) = c2(2,1);    % yC2
output(6,1) = c2(3,1);    % zC2
output(7,1) = c3(1,1);    % xC3
output(8,1) = c3(2,1);    % yC3
output(9,1) = c3(3,1);    % zC3

output(10,1) = theta13dd; % theta13dd
output(11,1) = theta23dd; % theta23dd
output(12,1) = w3ld;      % w3ld
output(13,1) = cod(1,1);  % xCod
output(14,1) = cod(2,1);  % yCod
output(15,1) = cod(3,1);  % zCod
output(16,1) = rd(1,1);   % rxd
output(17,1) = rd(2,1);   % ryd
output(18,1) = rd(3,1);   % rzd
output(19,1) = r0d;       % r0d

output(20,1) = rw/2*(theta13dd + theta23dd + 2*w3ld); % vcd
output(21,1) = verif_C1;
output(22,1) = verif_C2;
output(23,1) = verif_ERP2;