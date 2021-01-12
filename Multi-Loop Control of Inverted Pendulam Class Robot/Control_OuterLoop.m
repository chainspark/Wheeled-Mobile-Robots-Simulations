function output = Control_OuterLoop(v0, x0, y0, rx, ry, rz, r0, w3p, vc)

Parameters

% Inertial Frame
i = [1;0;0];

% Computation of Q3 = [u3 l v3]
r = [rx;ry;rz];
R = [0 -rz ry;rz 0 -rx;-ry rx 0];
Q3 = (r0^2 - r'*r)*eye(3) + 2*(r*r') + 2*r0*R;
l = Q3(:,2);

% Useful Substitutions
k = [0;0;1];
n = k;
s = cross(n,i);
h = cross(l,n);
B = 0;

% Gradient and Delta
ep_x = -x0/2;
ep_y = -y0;
ep_norm = sqrt(ep_x^2 + ep_y^2);

% Equation Sub-formula
D1 = h'*i*(ep_x/ep_norm) + h'*s*(ep_y/ep_norm);
D2 = ((x0*y0)/(4*ep_norm^3))*(h'*i.^2 - 2*cos(B)*h'*s.^2) + ...
    ((h'*i*h'*s)/(4*ep_norm^3))*(cos(B)*x0^2 - 2*y0^2);
delta_dot = -D1*w3p + D2*vc;

% Outputs
vc_out = ep_norm*v0*sign(h'*i*ep_x);
w3p_out = (1/D1)*(D2*vc - delta_dot);

% Output vector
output(1,1) = vc_out;
output(2,1) = w3p_out;