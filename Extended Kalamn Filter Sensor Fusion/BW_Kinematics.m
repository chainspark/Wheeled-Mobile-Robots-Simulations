function [sys,x0,str,ts] = BW_Kinematics(t,x,u,flag)

% This S-Function calls "Compute_BW_Kinematics" 

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
sizes.NumOutputs     = 4;  
sizes.NumInputs      = 6;   
sizes.DirFeedthrough = 1;   
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
str = [];
x0  = [];
ts  = [-1 0];   

function sys = mdlOutputs(t,x,u)
sys = Compute_BW_Kinematics(u(1),u(2),u(3),u(4),u(5),u(6));  

 
