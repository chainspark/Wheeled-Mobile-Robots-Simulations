function output = compute_BWkinematics(psid,rw_thetard,xCr,yCr,phi,psi)

% Inputs of the function "compute_BWkinematics" :  
% psid : Steering rate
% rw_thetard : Heading velocity
% xCr, yCr : x & y coordinates of Cr, the midpoint of the back-wheels axis 
% phi : Orientation of the platform w.r.t F0
% psi : Steering angle

ASR_Parameters

% Definition of the state vector 
% q1 = xCr; 
% q2 = yCr;
% q3 = phi; 
% q4 = psi; 

% Definition of the inputs
% u1 = psid;
% u2 = rw_thetard;

% Computation of h and l 
% h = [cos(q3);sin(q3)];
% l = [-sin(q3);cos(q3)];
% k = [0;0;1];

% Computation of vector qd
q1d = cos(phi)*rw_thetard;
q2d = sin(phi)*rw_thetard;
q3d =(1/a)*tan(psi)*rw_thetard;
q4d = psid;

% Output vector
output(1,1) = q1d;    % xCrd
output(2,1) = q2d;    % yCrd
output(3,1) = q3d;    % phid
output(4,1) = q4d;    % psid