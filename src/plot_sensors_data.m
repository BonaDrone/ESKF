clc; clear
format long;
%% Load raw data
% try catch structure for debugging
try
   data = csvread("../data/raw_data_19.csv");
catch
   % do nothing, just avoid throwing an error
end

timestamps = data(:,1);

figure;
% Plot accels
subplot(4,1,1);
hold on;
grid on;

accelx = data(:,2);
accely = data(:,3);
accelz = data(:,4);

tx = timestamps(accelx ~= 1000.0);
ty = timestamps(accely ~= 1000.0);
tz = timestamps(accelz ~= 1000.0);

accelx = accelx(accelx ~= 1000.0);
accely = accely(accely ~= 1000.0);
accelz = accelz(accelz ~= 1000.0);

plot(tx, accelx, 'b');
plot(ty, accely, 'r');
plot(tz, accelz, 'g');
title('accelerations')  
ylim([-0.2 1.2])

subplot(4,1,2);
grid on;
plot(tx, accelx, 'b');
title('x-axis acceleration (g)')  

subplot(4,1,3);
grid on;
plot(ty, accely, 'r');
title('y-axis acceleration (g)')

subplot(4,1,4);
grid on;
plot(tz, accelz, 'g');
title('z-axis acceleration (g)')

figure;
% Plot angular velocities
hold on;
grid on;
gyrox = data(:,5);
gyroy = data(:,6);
gyroz = data(:,7);

tx = timestamps(gyrox ~= 1000.0);
ty = timestamps(gyroy ~= 1000.0);
tz = timestamps(gyroz ~= 1000.0);

gyrox = gyrox(gyrox ~= 1000.0) * pi/180;
gyroy = gyroy(gyroy ~= 1000.0) * pi/180;
gyroz = gyroz(gyroz ~= 1000.0) * pi/180;

plot(tx, gyrox, 'b');
plot(ty, gyroy, 'r');
plot(tz, gyroz, 'g');
title('angular velocities (rad/s)')


figure;
% Plot range height
grid on;
range = data(:,8);

tr = timestamps(range ~= 1000.0);

range = range(range ~= 1000.0);

plot(tr, range, 'b');
title('height (m)')

figure;
% Plot flows
hold on;
grid on;
flowx = data(:,9);
flowy = data(:,10);

tx = timestamps(flowx ~= 1000.0);
ty = timestamps(flowy ~= 1000.0);

flowx = flowx(flowx ~= 1000.0);
flowy = flowy(flowy ~= 1000.0);

plot(tx, flowx, 'b');
plot(ty, flowy, 'r');
title('flows (accumulated pixels)')
