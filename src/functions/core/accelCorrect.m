function [x,P] = accelCorrect(x,inputArg2)
%ACCELCORRECT Summary of this function goes here
%   Detailed explanation goes here

    persistent N; N = [0.7, 0, 0; 0, 0.7, 0; 0, 0, 0.7];
    persistent g; g = 9.80665;
    
    % measurement model
    h = [ g*(2*x(7)*x(9) - 2*x(8)*x(10));
         -g*(2*x(7)*x(8) + 2*x(9)*x(10));
         -g*(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2)];
    
    % Jacobian of measurement model w.r.t. error states
    H = [ 0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0,                                   0,                     0;...
          0, 0, 0, 0, 0, 0,                                 0, - g*x(7)^2 + g*x(8)^2 + g*x(9)^2 - g*x(10)^2, 2*g*x(7)*x(8) + 2*g*x(9)*x(10);...
          0, 0, 0, 0, 0, 0, g*x(7)^2 - g*x(8)^2 - g*x(9)^2 + g*x(10)^2,                                   0, 2*g*x(7)*x(9) - 2*g*x(8)*x(10);...
          0, 0, 0, 0, 0, 0,           - 2*g*x(7)*x(8) - 2*g*x(9)*x(10),               2*g*x(8)*x(10) - 2*g*x(7)*x(9),                     0];

    x = x;
    P = inputArg2;
end

