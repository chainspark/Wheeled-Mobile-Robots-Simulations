function [sys,x0,str,ts] = InnerLoop(t,x,u,flag)

% This S-Function calls the "compute_dynamics" using 
% Euler-Rodrigues parameters  

switch flag,
case 0
   [sys,x0,str,ts]=mdlInitializeSizes;
case 3
   sys=mdlOutputs(t,x,u);
case { 1, 2, 4, 9 }
   sys=[];
otherwise
   error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes()
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;  
sizes.NumInputs      = 14;   
sizes.DirFeedthrough = 1;   
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
str = [];
x0  = [];
ts  = [-1 0];   

function sys = mdlOutputs(t,x,u)
sys = Control_InnerLoop(u(1),u(2),u(3),u(4),u(5),u(6),u(7),u(8),u(9),u(10),u(11),u(12),u(13),u(14));  