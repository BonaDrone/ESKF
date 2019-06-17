function [x, P] = updateState(x, P, y, q, dt)
%UPDATE state estimation and process covariance 
%
    persistent g; g = 9.80665;
    
    persistent Q; % Adjust Q

%     Q = blkdiag(zeros(3), 0.08*diag(ones(1,3)), diag(ones(1,3)));
    Q = blkdiag(zeros(3), 0.08*diag(ones(1,3)), 0.08*diag(ones(1,3)));%, diag(ones(1,3)));

%     Q(1,1) = 0; Q(2,2) = 0; Q(3,3) = 0;
%     Q(4,4) = 0.01; Q(5,5) = 0.01; Q(6,6) = 0.01;
%     Q(7,7) = 5.0; Q(8,8) = 5.0; Q(9,9) = 5.0;

    % Integration model: Update state
    as = y(1:3)';
%     ws = y(4:6)';
    ab = x(7:9);
    
    %q_aux = [1; ws*dt/2];
    %q = qProd(x(7:10), q_aux);
    v = x(4:6) + q2R(q)*(as - ab + [0;0;-g])*dt;    
    p = x(1:3) + v*dt;
        
    x = [p; v; ab];% q];
    
    % Error-State Jacobian
%     Fn =  [ 1, 0, 0, dt,  0,  0,                                                               0,                                                               0,                                                               0;...
%             0, 1, 0,  0, dt,  0,                                                               0,                                                               0,                                                               0;...
%             0, 0, 1,  0,  0, dt,                                                               0,                                                               0,                                                               0;...
%             0, 0, 0,  1,  0,  0,          dt*(y(2)*(2*x(7)*x(9) + 2*x(8)*x(10)) + y(3)*(2*x(7)*x(10) - 2*x(8)*x(9))),  dt*(y(3)*(x(7)^2 + x(8)^2 - x(9)^2 - x(10)^2) - y(1)*(2*x(7)*x(9) + 2*x(8)*x(10))), -dt*(y(2)*(x(7)^2 + x(8)^2 - x(9)^2 - x(10)^2) + y(1)*(2*x(7)*x(10) - 2*x(8)*x(9)));...
%             0, 0, 0,  0,  1,  0, -dt*(y(3)*(x(7)^2 - x(8)^2 + x(9)^2 - x(10)^2) + y(2)*(2*x(7)*x(8) - 2*x(9)*x(10))),          dt*(y(1)*(2*x(7)*x(8) - 2*x(9)*x(10)) + y(3)*(2*x(7)*x(10) + 2*x(8)*x(9))),  dt*(y(1)*(x(7)^2 - x(8)^2 + x(9)^2 - x(10)^2) - y(2)*(2*x(7)*x(10) + 2*x(8)*x(9)));...
%             0, 0, 0,  0,  0,  1,  dt*(y(2)*(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2) - y(3)*(2*x(7)*x(8) + 2*x(9)*x(10))), -dt*(y(1)*(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2) + y(3)*(2*x(7)*x(9) - 2*x(8)*x(10))),          dt*(y(1)*(2*x(7)*x(8) + 2*x(9)*x(10)) + y(2)*(2*x(7)*x(9) - 2*x(8)*x(10)));...
%             0, 0, 0,  0,  0,  0,                                                               1,                                                          dt*y(6),                                                         -dt*y(5);...
%             0, 0, 0,  0,  0,  0,                                                         -dt*y(6),                                                               1,                                                          dt*y(4);...
%             0, 0, 0,  0,  0,  0,                                                          dt*y(5),                                                         -dt*y(4),                                                               1];

%     V  = -q2R(q)*skew(as-ab);          % as -> measured accelerations
%     Fi = -skew(ws);                 % ws -> measured angular velocities
    R = q2R(q);
        
%     A_dx = [zeros(3)   eye(3)   zeros(3); ...
%             zeros(3)  zeros(3)    V     ; ...
%             zeros(3)  zeros(3)    Fi   ]; 

%     Fn = eye(9) + A_dx*dt;    

    A_dx = [zeros(3)   eye(3) zeros(3);...
            zeros(3)  zeros(3) -R     ;...
                  zeros(3,9)          ];

    Fn = eye(9) + A_dx*dt;    

    % Predict covariance
    P = Fn*P*Fn.' + Q; % Q is already Fi*Q*Fi.';
    
end
