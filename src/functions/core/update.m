function [x,P] = update(inputArg1,inputArg2)
%UPDATE Summary of this function goes here
%   Detailed explanation goes here

    persistent Q;

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
    % R  = fromqtoR(q);
    Fi = -skew(ws);
    
    A_dx = ...
        [zeros(3) eye(3)   zeros(3); ...
        zeros(3)  zeros(3) V       ; ...
        zeros(3)  zeros(3) Fi      ];
    
    F_dx = eye(9) + A_dx*dt;
    
    x = inputArg1;
    P = inputArg2;
end
