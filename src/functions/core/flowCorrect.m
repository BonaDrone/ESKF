function [x,P] = flowCorrect(inputArg1,inputArg2)
%FLOWCORRECT Summary of this function goes here
%   Detailed explanation goes here

    persistent N;
    
    %% h(x) - Optical Flow

    syms fx fy % camera's focal distances
    
    P_f = [fx 0 0; 0 fy 0];
    P_x = [0 fx 0; -fy 0 0];
    
    Rz = [0 -1 0; 1 0 0; 0 0 1];
    Ry = [-1 0 0; 0 1 0; 0 0 -1];
    
    % R_c_i = Rz*Ry;
    R_c_i = diag([1 -1 -1]);
    R = fromqtoR(q);
    
    p_c_w = p;
    v_c_w = v;
    R_c_w = R*R_c_i;
    
    v_c_c = R_c_i.'*R.'*v_c_w;
    w_c_c = R_c_i.'*(ws);
    z_c = p_c_w(3)/R_c_w(3,3);
    
    % measurement model as a function of system states
    h_f = -P_f*(v_c_c/z_c) - P_x*w_c_c;
    % jacobian of measurement model
    H_f = jacobian(h_f, x);
    X_dx = blkdiag(eye(6), Qmat(q));
    H_dx_f = H_f*X_dx;

    x = inputArg1;
    P = inputArg2;
end

