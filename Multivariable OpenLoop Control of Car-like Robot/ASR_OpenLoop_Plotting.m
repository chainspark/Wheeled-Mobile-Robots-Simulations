% *************************************************************************
% Ackermann Steering Robot Model Output Plotting
% Units = Meters
% NOTE: u1 = v (driving wheel velocity) & u2 = psi_dot (steering rate)
% *************************************************************************
% Plot X-Y
subplot(2,2,[1,3])
hold on
plot(x_out.Data, y_out.Data)
xlabel('X') 
ylabel('Y') 
title('Position')
hold off
% Plot Phi & Psi
subplot(2,2,2)
hold on
plot(phi_out.Time, phi_out.Data)
plot(psi_out.Time, psi_out.Data)
xlabel('Time')
legend('Phi','Psi')
title('Heading and Steering')
hold off
% Plot v1 & v2
subplot(2,2,4)
hold on
plot(v1_out.Time, v1_out.Data)
plot(v2_out.Time, v2_out.Data)
xlabel('Time')
legend('v1','v2')
title('Input')
hold off