clc; clear
%% Define States
syms px py pz
syms vx vy vz
syms qw qx qy qz
syms abx aby abz
syms wbx wby wbz

syms dpx dpy dpz
syms dvx dvy dvz
syms dwx dwy dwz
syms dabx daby dabz
syms dwbx dwby dwbz

syms g dt

% Nominal states
p  = [px; py; pz];
v  = [vx; vy; vz];
qv = [qx; qy; qz];
q  = [qw; qv];
ab = [abx; aby; abz];
wb = [wbx; wby; wbz];

x = [p; v; q; ab; wb];

gv = [0;0;-g];

% Error states
dp  = [dpx; dpy; dpz];
dv  = [dvx; dvy; dvz];
dw  = [dwx; dwy; dwz];
dab = [dabx; daby; dabz];
dwb = [dwbx; dwby; dwbz];

dx = [dp; dv; dw; dab; dwb];

% Measurements - IMU
syms asx asy asz
syms wsx wsy wsz

as = [asx; asy; asz];
ws = [wsx; wsy; wsz];

%% Integration model
F_x = x;
R = fromqtoR(q);
aux    = R*(as-ab) + [0;0;-g];
q_aux  = [1;(ws-wb)*dt/2];
F_x(1:3)   = p + v*dt;    % p <- p + v*dt
F_x(4:6)   = v + aux*dt;  % v <- v + (R(as-ab)+g)dt
F_x(7:10) = leftQuaternion(q)*q_aux; % q <- q x q((ws-wb)dt)
F_x(11:13) = ab; % ab <- ab
F_x(14:16) = wb; % wb <- wb

%% Nominal Jacobian - obsolete
% 
% V = fromqtoR(q)*(as-ab);
% V = jacobian(V, q);
% R = -fromqtoR(q);
% W = 0.5*[0, -transpose(ws-wb); ws-wb, -skew(ws-wb)];
% Q = -0.5*Qmat(q);
% 
% % gv = (as-ab);
% % 2*[qw*gv+cross(qv,gv), qv*transpose(gv)-gv*transpose(qv)+transpose(gv)*qv*eye(3)-qw*skew(gv)]
% 
% A_x = ...
%     [zeros(3) eye(3)   zeros(3, 4) zeros(3) zeros(3); ...
%     zeros(3) zeros(3) V           -R        zeros(3); ...
%     zeros(3) zeros(3) W           zeros(3) Q       ; ...
%     zeros(6, 16)];
% 
% F_x = eye(16) + A_x*dt;
% 
%% Error-State Jacobian

V  = -fromqtoR(q)*skew(as-ab);
R  = fromqtoR(q);
Fi = -skew(ws-wb);

A_dx = ...
    [zeros(3) eye(3)   zeros(3) zeros(3) zeros(3); ...
    zeros(3)  zeros(3) V        -R       zeros(3); ...
    zeros(3)  zeros(3) Fi       zeros(3) -eye(3);  ...
    zeros(6,15)];

F_dx = eye(15) + A_dx*dt;

%% h(x) - accel. Jacobian of h(x) w.r.t the quaternion

R_t = fromqtoR(q).';
h_a = R_t*gv;
H_x = jacobian(h_a, x);
X_dx = Qmat(q);
X_dx = blkdiag(eye(6), X_dx, eye(6));
H_dx = H_x*X_dx;


%% h(x) - Range Finder

% syms rx ry rz

R_r_i = [1 0 0; 0 -1 0; 0 0 -1];
R = fromqtoR(q);
p_r_w = -p; % Assume range and imu at same position (no offset).
            % The minus sign is because we want the measure to be
            % from drone to ground and not otherwise.
R_r_w = R*R_r_i;

% measurement model as a function of system states
h_r = p_r_w(3)/R_r_w(3,3);
% jacobian of measurement model
H_r = jacobian(h_r, x);
X_dx = Qmat(q);
X_dx = blkdiag(eye(6), X_dx, eye(6));
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
w_c_c = R_c_i.'*(ws-wb);
z_c = p_c_w(3)/R_c_w(3,3);

% measurement model as a function of system states
h_c = -P_f*(v_c_c/z_c) - P_x*w_c_c;
% jacobian of measurement model
H_c = jacobian(h_c, x);
X_dx = Qmat(q);
X_dx = blkdiag(eye(6), X_dx, eye(6));
H_dx_c = H_c*X_dx;

%% Another model that works with velocities instead of flows
syms d f K ofx ofy
h_c_2 = (d/30)*(pz/f)*K*[-ofx/dt; -ofy/dt] + pz*[-(wsy-wby); (wsx-wbx)];

H_c_2 = jacobian(h_c_2, x);
X_dx = Qmat(q);
X_dx = blkdiag(eye(6), X_dx, eye(6));
H_dx_c_2 = H_c_2*X_dx;

