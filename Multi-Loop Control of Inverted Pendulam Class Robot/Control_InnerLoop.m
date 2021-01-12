function output = Control_InnerLoop(u3z_ref,w3p_ref,theta13d,theta23d, ...
    w3l,cox,coy,coz,rx,ry,rz,r0,theta13,theta23)

Parameters

% Inertial frame and normal vector
i = [1;0;0];
j = [0;1;0];
k = [0;0;1];
n = k;  %!< On Horizontal Plane
nx = n'*i; ny = n'*j; nz = n'*k;
kxn = cross(k,n);

% Subsitutions
w3p = (rw/b)*(theta13d - theta23d);
vc = (rw/2)*(theta13d + theta23d + 2*w3l);

% Computation of Q3 = [u3 l v3]
r = [rx;ry;rz];
R = [0 -rz ry;rz 0 -rx;-ry rx 0];
Q3 = (2*r0^2-1)*eye(3) + 2*r*r' + 2*r0*R;
u3 = Q3(:,1); l = Q3(:,2); v3 = Q3(:,3);
u3z = u3'*k; v3z = v3'*k;
h = cross(l,n);
hi = h'*i; 

% xi1, xi2 and xi3
xi1 = u3z;
xi2 = (u3'*kxn)*w3p - v3z*w3l;
xi3 = w3p;
xi1_ref = u3z_ref;
xi3_ref = w3p_ref;

w3 = w3p*n + w3l*l;
u3d = cross(w3,u3); 
v3d = cross(w3,v3); 
hd = w3p*l;

% Useful Substitutions
hu3 = h'*u3;
hv3 = h'*v3;
hk = h'*k;
v3k = v3'*k;
v3xn = cross(v3,n);
v3xnk = v3xn'*k;

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
Gb = Jb*mc*g*d*u3z + (Jc-Jb)*(2*mw+mc)*g*rw*hk;
Gc = - Jc*mc*g*d*u3z - Jd*(2*mw+mc)*g*rw*hk;

% Control Layer 0: u3z & hx
d1 = u3d'*kxn*w3p - v3d'*k*w3l - v3k*(Fb+Gb) + (2*rw/b)*u3'*kxn*(Fa+Ga);
d2 = (2*rw/b)*(Fa+Ga);

%%%%%%%%%%%%%%  Controller Design   %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Choosing k1, k2 & k3 %%%%%%%%%%%%%%%%
k1 = 400;
k2 = 20;
k3 = 10;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

taum = -(d2 + k3*(xi3-xi3_ref))/(2*rw/b)/Ja;
taup = -(d1 + (2*rw/b)*Ja*u3'*kxn*taum + k2*xi2 + k1*(xi1-xi1_ref))/Jc/v3k;

tau1 = (taup + taum)/2;
tau2 = (taup - taum)/2;

% Computation of vcdss
ka = mc*g*rw*d/2/(Ic-If);
d3 = -(2*mw + mc)*g*rw^2*hk/2/(Ic-If);
vcdss = d3 + ka*u3z;


% Output vector
output(1,1) = tau1;
output(2,1) = tau2;
output(3,1) = u3z;
output(4,1) = ka;
output(5,1) = d3;
output(6,1) = vcdss;

