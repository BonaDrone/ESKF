function [x,P] = rangeCorrect(x,inputArg2)
%RANGECORRECT Summary of this function goes here
%   Detailed explanation goes here

    persistent N; N = 0.5;
    
    % measurement model
    h = x(3)/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2);

    % Jacobian of measurement model w.r.t error states
    H = [ 0, 0, 1/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2), 0, 0, 0, (2*x(3)*x(7)*x(8))/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2)^2 + (2*x(3)*x(9)*x(10))/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2)^2, (2*x(3)*x(7)*x(9))/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2)^2 - (2*x(3)*x(8)*x(10))/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2)^2, 0, 0, 0, 0, 0, 0, 0]; 
    
    x = x;
    P = inputArg2;
end

