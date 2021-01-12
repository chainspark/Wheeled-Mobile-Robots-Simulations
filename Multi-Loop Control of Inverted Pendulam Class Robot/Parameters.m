warning off

% Simulation parameters
Ts = 1e-1;

% Robot Parameters
mw = 0.75; 
m1 = mw;
m2 = mw;
m_cylinder = 4.25;  
m_rod = 4.25; 
mc = m_cylinder + m_rod;
m3 = mc;

g = 9.81;
cm_unit = 1/100;
rw = 7.75*cm_unit;   % radius of the wheel
b = 34.5*cm_unit;    % distance [C1C2]
rho = b/rw;
r_cylinder = 6*cm_unit;  % radius of the cylinder, should be less than r_wheel   
l_cylinder = 18*cm_unit; % length of the cylinder, should be less than (b_length - 2*r_wheel)
d_rod = -6*cm_unit;      % position of the rod, responsible of the offset of C3, 
                         % should be less than r_cylinder   
                         % if d_rod > 0 then the pendulum is non-inverted (exercise 1)
                         % if d_rod < 0 then the pendulum is inverted (exercises 2, 3 and 4)

% Inertial properties of the spherical wheel
Iwu = (2/5)*m1*rw^2;
Iwl = (2/5)*m1*rw^2;
Iwv = Iwu;

% Inertial properties of the central body
d = (m_rod/m3)*d_rod;
Icu = (1/12)*m_cylinder*(3*r_cylinder^2 + l_cylinder^2) + m_cylinder*d^2 + ...
    (1/12)*m_rod*l_cylinder^2 + m_rod*(d_rod-d)^2;
Icl = (1/2)*m_cylinder*r_cylinder^2 + m_cylinder*d^2 + m_rod*(d_rod - d)^2;
Icv = (1/12)*m_cylinder*(3*r_cylinder^2 + l_cylinder^2) + (1/12)*m_rod*l_cylinder^2;

Ia = 2*(rw/b)^2*Iwu + Iwl + mw*rw^2 + mc*rw^2/4;
Ib = -2*(rw/b)^2*Iwu + mc*rw^2/4;
Ic = Iwl + mw*rw^2 + mc*rw^2/2;
Id = Icl + mc*d^2;

M1 = diag([Iwu;Iwl;Iwv;m1;m1;m1]);
M2 = M1;
M3 = diag([Icu;Icl;Icv;m3;m3;m3]);