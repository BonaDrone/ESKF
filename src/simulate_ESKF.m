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

% Iterate through all rows of raw data. It is assumed here that
% sensor data has already been collected at the desired frequencies.
% This way, each iteration simulates the arrival of new data from
% a specific sensor
for i = 1 : length(data(:,1))
    timestamp = data(i, 1);
    sensor_data = data(i, 2:end);
    
    IMUData = sensor_data(1) ~= 1000.0 && sensor_data(2) ~= 1000.0 && sensor_data(3) ~= 1000.0; 
    accelData = sensor_data(1) ~= 1000.0 && sensor_data(2) ~= 1000.0 && sensor_data(3) ~= 1000.0; 
    rangeData = sensor_data(7) ~= 1000.0; 
    flowData = sensor_data(8) ~= 1000.0 && sensor_data(9) ~= 1000.0; 

    if (IMUData)
        disp("IMU Data");
        %x; P = update(x,P,sensor_data);
    end
    if (accelData)
        disp("Accel Data");
        %x; P = accelCorrect(x,P,sensor_data);
    end
    if (rangeData)
        disp("Range Data");
        %x; P = rangeCorrect(x,P,sensor_data);
    end
    if (flowData)
        disp("Flow Data");
        %x; P = flowCorrect(x,P,sensor_data);
    end

end


