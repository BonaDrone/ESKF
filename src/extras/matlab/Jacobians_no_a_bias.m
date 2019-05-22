clc; clear
%% Define States
syms px py pz
syms vx vy vz
syms qw qx qy qz
%syms abx aby abz
syms wbx wby wbz

syms dpx dpy dpz
syms dvx dvy dvz
syms dwx dwy dwz
%syms dabx daby dabz
syms dwbx dwby dwbz


syms g dt

% Nominal states
p  = [px; py; pz];
v  = [vx; vy; vz];
qv = [qx; qy; qz];
q  = [qw; qv];
%ab = [abx; aby; abz];
wb = [wbx; wby; wbz];

%x = [p; v; q; ab; wb];
x = [p; v; q; wb];

gv = [0;0;-g];

% Error states
dp  = [dpx; dpy; dpz];
dv  = [dvx; dvy; dvz];
dw  = [dwx; dwy; dwz];
%dab = [dabx; daby; dabz];
dwb = [dwbx; dwby; dwbz];

%dx = [dp; dv; dw; dab; dwb];
dx = [dp; dv; dw; dwb];

% Measurements - IMU
syms asx asy asz
syms wsx wsy wsz

as = [asx; asy; asz];
ws = [wsx; wsy; wsz];

%% Nominal Jacobian

%V = fromqtoR(q)*(as-ab);
V = fromqtoR(q)*as;
V = jacobian(V, q);
R = -fromqtoR(q);
W = 0.5*[0, -transpose(ws-wb); ws-wb, -skew(ws-wb)];
Q = -0.5*Qmat(q);

% gv = (as-ab);
% 2*[qw*gv+cross(qv,gv), qv*transpose(gv)-gv*transpose(qv)+transpose(gv)*qv*eye(3)-qw*skew(gv)]

%A_x = ...
%    [zeros(3) eye(3)   zeros(3, 4) zeros(3) zeros(3); ...
%    zeros(3) zeros(3) V           -R        zeros(3); ...
%    zeros(3) zeros(3) W           zeros(3) Q       ; ...
%    zeros(6, 16)];

A_x = ...
    [zeros(3) eye(3)   zeros(3, 4)  zeros(3); ...
    zeros(3) zeros(3) V             zeros(3); ...
    zeros(3) zeros(3) W             Q       ; ...
    zeros(3, 13)];

F_x = eye(13) + A_x*dt;

%% Error-State Jacobian

%V  = -fromqtoR(q)*skew(as-ab);
V = -fromqtoR(q)*skew(as);
R  = fromqtoR(q);
Fi = -skew(ws-wb);

A_dx = ...
    [zeros(3) eye(3)   zeros(3)  zeros(3); ...
    zeros(3)  zeros(3) V         zeros(3); ...
    zeros(3)  zeros(3) Fi        -eye(3);  ...
    zeros(3,12)];

F_dx = eye(12) + A_dx*dt;


%% h(x) - accel. Jacobian of h(x) w.r.t the quaternion

R_t = transpose(fromqtoR(q));
R_t_g = -R_t*gv;
H_x = jacobian(R_t_g, q);
H_x = blkdiag(zeros(6),H_x, zeros(6));
X_dx = Qmat(q);
X_dx = blkdiag(eye(6), X_dx, eye(6));
H_dx = H_x*X_dx;

%% h(x) - Range Finder
