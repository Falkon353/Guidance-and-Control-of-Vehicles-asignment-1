x = 0;
y = 100;
yaw = 0;
r = 0; 
U = 5;
b = 0.001;
T = 20;
K = 0.1;
K_p = 1;
K_d = 40;
K_i = 0.1;%0.00001;

sim('shipmoodel.slx')

figure (1); clf;
hold on;
plot(Controll_input)
hold off;
grid on;
legend('roder input');
title('Controll input');
xlabel('time [s]'); 
ylabel('angle [deg]');

figure (2); clf;
hold on;
plot(yaw_result)
hold off;
grid on;
legend('Yaw');
title('Yaw angel');
xlabel('time [s]'); 
ylabel('angle [deg]');

figure (3); clf;
hold on;
plot(yaw_rate)
hold off;
grid on;
legend('Yaw rate');
title('Yaw rate');
xlabel('time [s]'); 
ylabel('Angular velocities [deg/s]');

figure (4); clf;
hold on;
plot(Y_result.Data, X_result.Data);
hold off;
grid on;
legend('North East position');
title('North East position');
xlabel('East [m]'); 
ylabel('North [m]');




