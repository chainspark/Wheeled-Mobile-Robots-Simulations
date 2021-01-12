function x = System_Trans_Fcn(x,u)

dt = 0.05;
a  = 0.49;

x(1)=  x(1) + dt*cos(x(3))*u(1);
x(2) = x(2) + dt*sin(x(3))*u(1);
x(3) = x(3) + (1/a)*dt*tan(u(2))*u(1);

end