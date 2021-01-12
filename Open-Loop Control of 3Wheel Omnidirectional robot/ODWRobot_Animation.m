x=ODWRobot_Outputs(:,1);
y=ODWRobot_Outputs(:,2);
phi=ODWRobot_Outputs(:,3);

figure(1)
title('Platform center');
comet(x,y);
