clc; clear
format long;
%% Load raw data
% try catch structure for debugging
try
   data = csvread("../data/raw_data.csv");
catch
   % do nothing, just avoid throwing an error
end

figure;
% Plot accels
subplot(4,1,1);
hold on;
grid on;
plot(data(:,1), data(:,2), 'b');
plot(data(:,1), data(:,3), 'r');
plot(data(:,1), data(:,4), 'g');
title('accelerations')  

subplot(4,1,2);
grid on;
plot(data(:,1), data(:,2), 'b');
title('x-axis acceleration (g)')  

subplot(4,1,3);
grid on;
plot(data(:,1), data(:,3), 'r');
title('y-axis acceleration (g)')

subplot(4,1,4);
grid on;
plot(data(:,1), data(:,4), 'g');
title('z-axis acceleration (g)')

figure;
% Plot angular velocities
hold on;
grid on;
plot(data(:,1), data(:,5), 'b');
plot(data(:,1), data(:,6), 'r');
plot(data(:,1), data(:,7), 'g');
title('angular velocities (deg/s)')


figure;
% Plot range height
subplot(2,1,1)
grid on;
plot(data(:,1), data(:,8), 'b');
title('height (m)')


% Plot flows
subplot(2,1,2)
hold on;
grid on;
plot(data(:,1), data(:,9), 'b');
plot(data(:,1), data(:,10), 'r');
title('flows (accumulated pixels)')
