%% Start of script
addpath('madgwick/quaternion_library');     % include quaternion library
close all;                                  % close all figures
clear;                                      % clear all variables
clc;                                        % clear the command terminal
format long;
%% Load data

% try catch structure for debugging
try
   data = csvread("../data/raw_data.csv");
catch
   % do nothing, just avoid throwing an error
end
data = data(100:end, :);

data = data(data(:, 2) ~= 1000.0, :);
data = data(data(:, 5) ~= 1000.0, :);

time = data(:,1);

%% Process sensor data through algorithm

% see: https://github.com/BonaDrone/Hackflight/blob/06fe35f618ebedb435cff61db9bef9515d851366/src/boards/softquat.hpp#L36
b = sqrt(3.0 / 4.0) * pi * (40.0 / 180.0);

AHRS = MadgwickAHRS('SamplePeriod', 1/190, 'Beta', b);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

quaternion = zeros(length(time), 4);

for t = 2:length(time)
    dt = time(t) - time(t-1);
    AHRS.UpdateIMU(data(t, 5:7)*(pi/180), data(t, 2:4), dt);	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches �90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,1), 'b');
plot(time, euler(:,2), 'r');
plot(time, euler(:,3), 'g');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

figure;
% plot quaternion estimation
hold on;
grid on;
plot(time, quaternion(:,1)', 'k');
plot(time, quaternion(:,2)', 'b');
plot(time, quaternion(:,3)', 'r');
plot(time, quaternion(:,4)', 'g');
title('Quaternion')

%% End of script