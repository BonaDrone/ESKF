clc; clear
%% Define States
syms pz
syms vz
syms qw qx qy qz
syms wbx wby wbz

syms dpz
syms dvz
syms dwx dwy dwz
syms dwbx dwby dwbz

syms g dt

% Nominal states
p  = pz;
v  = vz;
qv = [qx; qy; qz];
q  = [qw; qv];
wb = [wbx; wby; wbz];

%x = [p; v; q; ab; wb];
x = [p; v; q; wb];

gv = [0;0;-g];

% Error states
dp  = dpz;
dv  = dvz;
dw  = [dwx; dwy; dwz];
dwb = [dwbx; dwby; dwbz];

dx = [dp; dv; dw; dwb];

% Measurements - IMU
syms asx asy asz
syms wsx wsy wsz

as = [asx; asy; asz];
ws = [wsx; wsy; wsz];

% Aux vars
syms rx ry rz

%% Integration model
F_x = x;
R = fromqtoR(q);
aux    = R*as + [0;0;-g];
q_aux  = [1;(ws-wb)*dt/2];
F_x(1)   = pz + vz*dt;
F_x(2)   = vz + aux(3)*dt;
F_x(3:6) = leftQuaternion(q)*q_aux;
F_x(7:9) = wb;

%% Nominal Jacobian - No longer used

%V = fromqtoR(q)*(as-ab);
%V = fromqtoR(q)*as;
%V = jacobian(V, q);
%R = -fromqtoR(q);
%W = 0.5*[0, -transpose(ws-wb); ws-wb, -skew(ws-wb)];
%Q = -0.5*Qmat(q);

%A_x = ...
%    [0               1    zeros(1, 4)  zeros(1, 3); ...
%     0               0      V(3,:)     zeros(1,3); ...
%    zeros(4,1)   zeros(4,1)    W             Q       ; ...
%    zeros(3, 9)];

%F_x = eye(9) + A_x*dt;

%% Error-State Jacobian

%V  = -fromqtoR(q)*skew(as-ab);
V = -fromqtoR(q)*skew(as);
R  = fromqtoR(q);
Fi = -skew(ws-wb);

A_dx = ...
    [0              1      zeros(1, 3)  zeros(1, 3); ...
     0              0         V(3,:)    zeros(1, 3); ...
    zeros(3, 1)  zeros(3,1)     Fi        -eye(3);  ...
    zeros(3, 8)];

F_dx = eye(8) + A_dx*dt;


%% h(x) - accel. Jacobian of h(x) w.r.t the quaternion

R_t = transpose(fromqtoR(q));
R_t_g = -R_t*gv;
H_x = jacobian(R_t_g, q);
H_x = [zeros(3,2), H_x, zeros(3,3)];
X_dx = Qmat(q);
X_dx = blkdiag(eye(2), X_dx, eye(3));
H_dx_a = H_x*X_dx;

%% h(x) - Range Finder

R_r_i = eye(3);
%p_r_i = [rx; ry; rz]; % measure distances
p_r_i = [0; 0; 0]; % measure distances
R = fromqtoR(q);

p_r_w = [0; 0; pz] + R*p_r_i;
R_r_w = R*R_r_i;

%h_r = p_r_w(3)/R_r_w(3,3); 
h_r = p_r_w(3);
H_r = jacobian(h_r, x);
X_dx = Qmat(q);
X_dx = blkdiag(eye(2), X_dx, eye(3));
H_dx_r = H_r*X_dx;