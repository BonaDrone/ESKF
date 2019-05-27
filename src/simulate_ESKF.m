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
x = zeros(10, 1); x(7) = 1.0;
P = diag(ones(1,9));

X = [];

% Iterate through all rows of raw data. It is assumed here that
% sensor data has already been collected at the desired frequencies.
% This way, each iteration simulates the arrival of new data from
% a specific sensor
for i = 2 : length(data(:,1))
    timestamp = data(i, 1);
    sensor_data = data(i, 2:end);
    
    % We can know if sensor data is available via the index and the value
    IMUData = sensor_data(1) ~= 1000.0 && sensor_data(2) ~= 1000.0 && sensor_data(3) ~= 1000.0; 
    accelData = sensor_data(1) ~= 1000.0 && sensor_data(2) ~= 1000.0 && sensor_data(3) ~= 1000.0; 
    rangeData = sensor_data(7) ~= 1000.0; 
    flowData = sensor_data(8) ~= 1000.0 && sensor_data(9) ~= 1000.0; 

    sensor_data(1) = sensor_data(1) * 9.80665;
    sensor_data(2) = sensor_data(2) * 9.80665;
    sensor_data(3) = sensor_data(3) * 9.80665;
    
    dt = timestamp - data(i-1,1);
    
    if (IMUData)
        %disp("IMU Data");
        [x, P] = updateState(x, P, sensor_data, dt);
    end
    if (accelData)
        %disp("Accel Data");
        [x, P] = accelCorrect(x, P, sensor_data);
    end
    if (rangeData)
        %disp("Range Data");
        [x, P] = rangeCorrect(x, P, sensor_data);
    end
    if (flowData)
        %disp("Flow Data");
        [x, P] = flowCorrect(x, P, sensor_data);
    end
    
    X = [X, x];

end

%% Plot state evolution

figure;
% plot position estimation
%subplot(4,1,1);
hold on;
grid on;
plot(data(2:end,1), X(1,:)', 'b');
plot(data(2:end,1), X(2,:)', 'r');
plot(data(2:end,1), X(3,:)', 'g');
title('Positions')

figure;
% plot velocity estimation
%subplot(4,1,1);
hold on;
grid on;
plot(data(2:end,1), X(4,:)', 'b');
plot(data(2:end,1), X(5,:)', 'r');
plot(data(2:end,1), X(6,:)', 'g');
title('Velocities')

figure;
% plot quaternion estimation
%subplot(4,1,1);
hold on;
grid on;
plot(data(2:end,1), X(7,:)', 'k');
plot(data(2:end,1), X(8,:)', 'b');
plot(data(2:end,1), X(9,:)', 'r');
plot(data(2:end,1), X(10,:)', 'g');
title('Quaternion')


