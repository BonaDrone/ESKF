addpath('madgwick/quaternion_library');     % include quaternion library
clc; 
clear;
format long;

%% Load raw data
% try catch structure for debugging
try
   data = csvread("../data/raw_data_59.csv");
catch
   % do nothing, just avoid throwing an error
end

% data = data(500:end, :);

%% ESKF Simulation

% init Madgwick filter to estimate the orientation
% see: https://github.com/BonaDrone/Hackflight/blob/06fe35f618ebedb435cff61db9bef9515d851366/src/boards/softquat.hpp#L36
%b = sqrt(3.0 / 4.0) * pi * (40.0 / 180.0); % roughly 0.6
b = 0.5;
AHRS = MadgwickAHRS('Beta', b);


% Flow Outlier detection
FLOW_LIMIT = 1000;
% Initialize state and P
x = zeros(6, 1);
%P = zeros(9,9);
P = 0.5*diag(ones(1,6));

X = [];

% Iterate through all rows of raw data. It is assumed here that
% sensor data has already been collected at the desired frequencies.
% This way, each iteration simulates the arrival of new data from
% a specific sensor and updates/corrects the state accordingly
previousFlowTimestamp = 1;

IMU_ACCEL_RATIO = 3;
counter = 0;
initFilter = 1;
firstIter = 0;

for i = 2 : length(data(:,1))
    % Separate sensor data and arrival time
    timestamp = data(i, 1);
    sensor_data = data(i, 2:end); 
    
    % We can know which sensor is providing us with new data via the index and the value
    IMUData = (sensor_data(1) ~= 1000.0) && (sensor_data(2) ~= 1000.0) && (sensor_data(3) ~= 1000.0) && (sensor_data(4) ~= 1000.0) && (sensor_data(5) ~= 1000.0) && (sensor_data(6) ~= 1000.0); 
    % accelData = (sensor_data(1) ~= 1000.0 && sensor_data(2) ~= 1000.0 && sensor_data(3) ~= 1000.0) && ~IMUData; 
    rangeData = sensor_data(10) ~= 1000.0; 
    flowData = sensor_data(11) ~= 1000.0 && sensor_data(12) ~= 1000.0;

    % Map sensor data to the units the filter expects
    sensor_data(1) = sensor_data(1) * 9.80665;
    sensor_data(2) = sensor_data(2) * 9.80665;
    sensor_data(3) = sensor_data(3) * 9.80665;
    
    sensor_data(4) = sensor_data(4) * pi / 180;
    sensor_data(5) = sensor_data(5) * pi / 180;
    sensor_data(6) = sensor_data(6) * pi / 180;
    
    if (~IMUData && (initFilter ~= 0))
        continue;    
    end
    
    if (IMUData)
        if (counter < IMU_ACCEL_RATIO)
            dt = timestamp - data(i-1,1);

%             AHRS.UpdateIMU(sensor_data(4:6), sensor_data(1:3), dt);
            AHRS.Update(sensor_data(4:6), sensor_data(1:3), sensor_data(7:9), dt);
            q = AHRS.Quaternion;

            [x, P] = updateStateM(x, P, sensor_data, q, dt);
            lastIMUData = sensor_data(1:9);
            counter = counter + 1;
        else
            dt = timestamp - data(i-1,1);

%             AHRS.UpdateIMU(lastIMUData(4:6), lastIMUData(1:3), dt);
            AHRS.Update(lastIMUData(4:6), lastIMUData(1:3), lastIMUData(7:9), dt);
            q = AHRS.Quaternion;

            [x, P] = updateStateM(x, P, lastIMUData, q, dt);
            %[x, P] = accelCorrectM(x, P, sensor_data, q);
            counter = 0;
        end
        
        if (initFilter == 1)
            initFilter = 0;
            firstIter = i;
        end
        
    else
        dt = timestamp - data(i-1,1);

%         AHRS.UpdateIMU(lastIMUData(4:6), lastIMUData(1:3), dt);        
        AHRS.Update(lastIMUData(4:6), lastIMUData(1:3), lastIMUData(7:9), dt);
        q = AHRS.Quaternion;

        [x, P] = updateStateM(x, P, lastIMUData, q, dt);
    end
    if (rangeData)
        [x, P] = rangeCorrectM(x, P, sensor_data, q);
    end
    if (flowData && abs(sensor_data(11)) < FLOW_LIMIT && abs(sensor_data(12)) < FLOW_LIMIT)
        dt = timestamp - data(previousFlowTimestamp,1);
        % If there is no IMU data then sensor_data(1:6) = 1000.0. Since
        % the Optical Flow correction model makes use of the angular
        % velocities to predict the number of pixels we set the first 6
        % values of the sensor data array (IMU Data) to the last IMU
        % measurements to prevent 1000 being used as the values of angular
        % velocities.
        if (~ IMUData)
            sensor_data(1:9) = lastIMUData;
        end
%         sensor_data(8) = -sensor_data(8)/dt;
%         sensor_data(9) = sensor_data(9)/dt;
%         [x, P] = flowCorrect(x, P, sensor_data);
          [x, P] = flowCorrectCrazyflieM(x, P, sensor_data, q, dt);
        previousFlowTimestamp = i;
    end
    x_c = [x; q.']; 
    X = [X, x_c]; % Log state after estimations

end

%% After the simulation, plot the state evolution

figure;
% plot position estimation
hold on;
grid on;
plot(data(firstIter:end,1), X(1,:)', 'b');
plot(data(firstIter:end,1), X(2,:)', 'r');
plot(data(firstIter:end,1), X(3,:)', 'g');
title('Positions')

figure;
% plot trajectory estimation
hold on;
grid on;
plot(X(1,:)', X(2,:), 'k');
% This might need some tuning
ylim([-2 2])
xlim([-2 2])
title('2D trajectory')

figure;
% plot trajectory estimation
hold on;
grid on;
plot3(X(1,:), X(2,:), X(3,:));
% This might need some tuning
ylim([-2 2])
xlim([-2 2])
zlim([0 1])
view(45,45)
title('3D trajectory')

figure;
% plot velocity estimation
hold on;
grid on;
plot(data(firstIter:end,1), X(6,:)', 'g');
plot(data(firstIter:end,1), X(5,:)', 'r');
plot(data(firstIter:end,1), X(4,:)', 'b');
title('Velocities')

figure;
% plot velocities estimation in imu frame
quats = X(7:10,:);
vels = [];
for i = 1 : length(quats(1,:))
    R = q2R(quats(:, i));
    vels = [vels, R*[X(4,i); X(5,i);X(6,i)]];
end
hold on;
grid on;
plot(data(firstIter:end,1), vels(1,:)', 'b');
plot(data(firstIter:end,1), vels(2,:)', 'r');
plot(data(firstIter:end,1), vels(3,:)', 'g');
title('Local velocities')


figure;
% plot quaternion estimation
hold on;
grid on;
plot(data(firstIter:end,1), X(7,:)', 'k');
plot(data(firstIter:end,1), X(8,:)', 'b');
plot(data(firstIter:end,1), X(9,:)', 'r');
plot(data(firstIter:end,1), X(10,:)', 'g');
title('Quaternion')

figure;
% plot euler angles' estimation
quats = X(7:10,:);
E = [];
for i = 1 : length(quats(1,:))
    e = q2e(quats(:, i));
    E = [E, e];
end
hold on;
grid on;
plot(data(firstIter:end,1), E(1,:)', 'b');
plot(data(firstIter:end,1), E(2,:)', 'r');
plot(data(firstIter:end,1), E(3,:)', 'g');
title('Euler angles')

