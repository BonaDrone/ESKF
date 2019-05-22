function [x,P] = rangeCorrect(inputArg1,inputArg2)
%RANGECORRECT Summary of this function goes here
%   Detailed explanation goes here

    persistent N;
    
    %% h(x) - Rangefinder Jacobian

    R_r_i = eye(3);
    p_r_i = [0; 0; 0]; % measure distances
    % p_r_i = [rx; ry; rz]; % measure distances
    R = fromqtoR(q);
    
    p_r_w = [0; 0; pz] + R*p_r_i;
    R_r_w = R*R_r_i;
    
    h_r = p_r_w(3)/R_r_w(3,3);
    
    H_r = jacobian(h_r, x);
    X_dx = blkdiag(eye(6), Qmat(q));
    
    H_dx_r = H_r*X_dx;

    x = inputArg1;
    P = inputArg2;
end

