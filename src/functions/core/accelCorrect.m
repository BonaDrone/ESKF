function [x,P] = accelCorrect(inputArg1,inputArg2)
%ACCELCORRECT Summary of this function goes here
%   Detailed explanation goes here
    persistent N;
    
    %% h(x) - accel. Jacobian of h(x) w.r.t the quaternion
    
    R_t = transpose(fromqtoR(q));
    R_t_g = -R_t*gv;
    
    H_x = blkdiag(zeros(6), jacobian(R_t_g, q));
    X_dx = blkdiag(eye(6), Qmat(q));
    
    H_dx_a = H_x*X_dx;

    x = inputArg1;
    P = inputArg2;
end

