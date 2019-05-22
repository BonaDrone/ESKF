clc; clear
%% Constants definitions to improve readability
IMU   = 1;
ACCEL = 2;
RANGE = 3;
FLOW  = 4;

%% Load raw data
% try catch structure for debugging
try
   data = csvread("../data/raw_data.csv");
catch
   % do nothing, just avoid throwing an error
end

%% Define States
syms px py pz
syms vx vy vz
syms qw qx qy qz

syms dpx dpy dpz
syms dvx dvy dvz
syms dwx dwy dwz

syms g dt

% Nominal states
p  = [px; py; pz];
v  = [vx; vy; vz];
qv = [qx; qy; qz];
q  = [qw; qv];

x = [p; v; q;];

gv = [0;0;-g];

% Error states
dp  = [dpx; dpy; dpz];
dv  = [dvx; dvy; dvz];
dw  = [dwx; dwy; dwz];

dx = [dp; dv; dw];

% Measurements - IMU
syms asx asy asz
syms wsx wsy wsz

as = [asx; asy; asz];
ws = [wsx; wsy; wsz];

%% ESKF Simulation
% Initialize state and P
x = zeros(10, 1); x(7) = 1.0;
P = diag(ones(1,9));

% Iterate through all rows of raw data. It is assumed here that
% sensor data has already been collected at the desired frequencies.
% This way, each iteration simulates the arrival of new data from
% a specific sensor
for i = 1 : length(data(:,1))
    sensor = data(i,2);
    timestamp = data(i, 1);
    new_data = data(i, 3:end);

    switch sensor
        case IMU % IMU data, update state estimation
            disp('IMU')
            x; P = update();
        case ACCEL % accel data, correct state
            disp('accel')
            x; P = accelCorrect();
        case RANGE % range data, correct state
            disp('range')
            x; P = rangeCorrect();
        case FLOW % flow data, correct state
            disp('flow')
            x; P = flowCorrect();
        otherwise
            disp('unknown data')
    end
end


