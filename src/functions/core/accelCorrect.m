function [x,P] = accelCorrect(x,inputArg2)
%ACCELCORRECT Summary of this function goes here
%   Detailed explanation goes here

    persistent N; N = [0.7, 0, 0; 0, 0.7, 0; 0, 0, 0.7];
    persistent g; g = 9.8665;
    
    % measurement model
    h = [ g*(2*x(7)*x(9) - 2*x(8)*x(10));
         -g*(2*x(7)*x(8) + 2*x(9)*x(10));
         -g*(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2)];
    
    % Jacobian of measurement model 
    H = [ 0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0, - g*x(7)^2 + g*x(8)^2 + g*x(9)^2 - g*x(10)^2, 2*g*x(7)*x(8) + 2*g*x(9)*x(10);...
          0, 0, 0, 0, 0, 0, g*x(7)^2 - g*x(8)^2 - g*x(9)^2 + g*x(10)^2,                                   0, 2*g*x(7)*x(9) - 2*g*x(8)*x(10);...
          0, 0, 0, 0, 0, 0,           - 2*g*x(7)*x(8) - 2*g*x(9)*x(10),               2*g*x(8)*x(10) - 2*g*x(7)*x(9),                     0];
    
    % h(x) - accel. Jacobian of h(x) w.r.t the quaternion
    
    % R_t = transpose(fromqtoR(q));
    % R_t_g = -R_t*gv;
    %     
    % H_x = blkdiag(zeros(6), jacobian(R_t_g, q));
    % X_dx = blkdiag(eye(6), Qmat(q));
    %     
    % H_dx_a = H_x*X_dx;

    x = inputArg1;
    P = inputArg2;
end

