clc; clear
format long;
%% Load raw data
% try catch structure for debugging
try
   data = csvread("../data/raw_data.csv");
catch
   % do nothing, just avoid throwing an error
end

%% ESKF Simulation
% Initialize state and P
x = zeros(10, 1); x(7) = 1.0; x(3) = 0.7;
P = zeros(9,9);
%P = diag(ones(1,9));

X = [];

% Iterate through all rows of raw data. It is assumed here that
% sensor data has already been collected at the desired frequencies.
% This way, each iteration simulates the arrival of new data from
% a specific sensor
previousTimestamp = 1;

for i = 2 : length(data(:,1))
    timestamp = data(i, 1);
    sensor_data = data(i, 2:end);
    
    % We can know if sensor data is available via the index and the value
    IMUData = (sensor_data(1) ~= 1000.0) && (sensor_data(2) ~= 1000.0) && (sensor_data(3) ~= 1000.0) && (sensor_data(4) ~= 1000.0) && (sensor_data(5) ~= 1000.0) && (sensor_data(6) ~= 1000.0); 
    accelData = (sensor_data(1) ~= 1000.0 && sensor_data(2) ~= 1000.0 && sensor_data(3) ~= 1000.0) && ~IMUData; 
    rangeData = sensor_data(7) ~= 1000.0; 
    flowData = sensor_data(8) ~= 1000.0 && sensor_data(9) ~= 1000.0;

    sensor_data(1) = sensor_data(1) * 9.80665;
    sensor_data(2) = sensor_data(2) * 9.80665;
    sensor_data(3) = sensor_data(3) * 9.80665;
    
    if (IMUData)
        dt = timestamp - data(previousTimestamp,1);
        [x, P] = updateState(x, P, sensor_data, dt);
        previousTimestamp = i;
    end
    if (accelData)
        [x, P] = accelCorrect(x, P, sensor_data);
    end
    if (rangeData)
        [x, P] = rangeCorrect(x, P, sensor_data);
    end
    if (flowData)
        [x, P] = flowCorrect(x, P, sensor_data);
    end
    
    X = [X, x]; % Log state after estimations

end

%% Plot state evolution

figure;
% plot position estimation
hold on;
grid on;
plot(data(2:end,1), X(1,:)', 'b');
plot(data(2:end,1), X(2,:)', 'r');
plot(data(2:end,1), X(3,:)', 'g');
title('Positions')

figure;
% plot velocity estimation
hold on;
grid on;
plot(data(2:end,1), X(4,:)', 'b');
plot(data(2:end,1), X(5,:)', 'r');
plot(data(2:end,1), X(6,:)', 'g');
title('Velocities')

figure;
% plot velocities estimation in imu frame
quats = X(7:10,:);
vels = [];
for i = 1 : length(quats(1,:))
    quats(:, i)
    R = q2R(quats(:, i));
    vels = [vels, R*[X(4,i); X(5,i);X(6,i)]];
end
hold on;
grid on;
plot(data(2:end,1), vels(1,:)', 'b');
plot(data(2:end,1), vels(2,:)', 'r');
plot(data(2:end,1), vels(3,:)', 'g');
title('Local velocities')


figure;
% plot quaternion estimation
hold on;
grid on;
plot(data(2:end,1), X(7,:)', 'k');
plot(data(2:end,1), X(8,:)', 'b');
plot(data(2:end,1), X(9,:)', 'r');
plot(data(2:end,1), X(10,:)', 'g');
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
plot(data(2:end,1), E(1,:)', 'b');
plot(data(2:end,1), E(2,:)', 'r');
plot(data(2:end,1), E(3,:)', 'g');
title('Euler angles')

