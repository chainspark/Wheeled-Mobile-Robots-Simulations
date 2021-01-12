%% 1. Open Loop Simulation
tiledlayout(1,3)
% Co Plot
Co_plt = nexttile;
plot(Co_plt,xCo,yCo)
title(Co_plt,'Co')
% C1 Plot
C1_plt = nexttile;
plot(C1_plt,xC1,yC1)
title(C1_plt,'C1')
% C2 Plot
C2_plt = nexttile;
plot(C2_plt,xC2,yC2)
title(C2_plt,'C2')
%% 2. Internal Loop
figure
% Plot u3zfigure
subplot(1,2,1)
hold on
plot(tout, u3z, '-r')
plot(tout, u3z_ref, '-b')
legend('u3z','u3z ref')
hold off
% Plot w3p
subplot(1,2,2)
hold on
plot(tout, w3p, '-r')
plot(tout, w3p_ref, '-b')
legend('w3p','w3p ref')
hold off
%% 3. Intermediate Loop
kv = -2.68;
tauv = 0.05;
% Plot vc
figure
hold on
plot(tout, vc_ref, '-r')
plot(tout, vc, '-b')
legend('vc Ref', 'vc')
hold off
%% 4. Outer Loop
% Auxiliary Input
v0  = 0.2;
% Initiate Animation
axis vis3d
axis([-0.5 1 0 1 -0.1 0.25])
grid on
view(70, 10)
hold on
hRobot = plot3([xC1_init xC2_init xCo_init xC3_init], [yC1_init yC2_init yCo_init yC3_init], ...
    [zC1_init zC2_init zCo_init zC3_init],'k.-', 'linewidth',4,'markersize',15);
% Animate
GraphingTimeDelay = 0.05;
for i = 1:length(tout)
    set(hRobot,'xdata',[xC1(i) xC2(i) xCo(i) xC3(i)]',...
        'ydata',[yC1(i) yC2(i) yCo(i) yC3(i)]',...
        'zdata',[zC1(i) zC2(i) zCo(i) zC3(i)*d_rod]');
    pause(GraphingTimeDelay);
end