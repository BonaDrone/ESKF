function [x,P] = rangeCorrect(x,inputArg2)
%RANGECORRECT Summary of this function goes here
%   Detailed explanation goes here

    persistent N;
    
    % measurement model
    h = x(3)/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2);

    % Jacobianof measurement model w.r.t error states
    H = [ 0, 0, 1/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2), 0, 0, 0, (2*x(3)*x(7)*x(8))/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2)^2 + (2*x(3)*x(9)*x(10))/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2)^2, (2*x(3)*x(7)*x(9))/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2)^2 - (2*x(3)*x(8)*x(10))/(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2)^2, 0, 0, 0, 0, 0, 0, 0];
 
    
    %% h(x) - Rangefinder Jacobian
% 
%     R_r_i = eye(3);
%     p_r_i = [0; 0; 0]; % measure distances
%     % p_r_i = [rx; ry; rz]; % measure distances
%     R = fromqtoR(q);
%     
%     p_r_w = [0; 0; x(3)] + R*p_r_i;
%     R_r_w = R*R_r_i;
%     
%     h_r = p_r_w(3)/R_r_w(3,3);
%     
%     H_r = jacobian(h_r, x);
%     X_dx = blkdiag(eye(6), Qmat(q));
%     
%     H_dx_r = H_r*X_dx;

    x = x;
    P = inputArg2;
end

