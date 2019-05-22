clc; clear
%% Load raw data
% try catch structure for debugging
try
   data = csvread("../data/raw_data.csv");
catch
   % do nothing
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

%% Integration model
F_x = x;
R = fromqtoR(q);
aux    = R*as + [0;0;-g];
q_aux  = [1; ws*dt/2];
F_x(1:3)   = p + v*dt;    % p <- p + v*dt
F_x(4:6)   = v + aux*dt;  % v <- v + (R*as+g)*dt
F_x(7:10) = leftQuaternion(q)*q_aux; % q <- q x q(ws*dt)

%% Error-State Jacobian

V  = -fromqtoR(q)*skew(as);
R  = fromqtoR(q);
Fi = -skew(ws);

A_dx = ...
    [zeros(3) eye(3)   zeros(3); ...
    zeros(3)  zeros(3) V       ; ...
    zeros(3)  zeros(3) Fi      ];

F_dx = eye(9) + A_dx*dt;

%% h(x) - accel. Jacobian of h(x) w.r.t the quaternion

R_t = transpose(fromqtoR(q));
R_t_g = -R_t*gv;

H_x = blkdiag(zeros(6), jacobian(R_t_g, q));
X_dx = blkdiag(eye(6), Qmat(q));

H_dx_a = H_x*X_dx;

%% h(x) - Rangefinder Jacobian

R_r_i = eye(3);
p_r_i = [0; 0; 0]; % measure distances
%p_r_i = [rx; ry; rz]; % measure distances
R = fromqtoR(q);

p_r_w = [0; 0; pz] + R*p_r_i;
R_r_w = R*R_r_i;

h_r = p_r_w(3)/R_r_w(3,3);

H_r = jacobian(h_r, x);
X_dx = blkdiag(eye(6), Qmat(q));

H_dx_r = H_r*X_dx;

%% h(x) - Optical Flow

syms fx fy % camera's focal distances

P_f = [fx 0 0; 0 fy 0];
P_x = [0 fx 0; -fy 0 0];

Rz = [0 -1 0; 1 0 0; 0 0 1];
Ry = [-1 0 0; 0 1 0; 0 0 -1];

%R_c_i = Rz*Ry;
R_c_i = diag([1 -1 -1]);
R = fromqtoR(q);

p_c_w = p;
v_c_w = v;
R_c_w = R*R_c_i;

v_c_c = R_c_i.'*R.'*v_c_w;
w_c_c = R_c_i.'*(ws);
z_c = p_c_w(3)/R_c_w(3,3);

% measurement model as a function of system states
h_f = -P_f*(v_c_c/z_c) - P_x*w_c_c;
% jacobian of measurement model
H_f = jacobian(h_f, x);
X_dx = blkdiag(eye(6), Qmat(q));
H_dx_f = H_f*X_dx;


