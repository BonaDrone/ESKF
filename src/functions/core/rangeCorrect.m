function [x,P] = rangeCorrect(x,P,y)
%RANGECORRECT Correct state estimation with range data
%

    persistent N; N = 0.01;
    
    p = x(1:3);
    q = x(7:10);
    R = q2R(q);
    
    % measurement model
    h = p(3)/R(3,3); % R(3,3) = q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2;
    
    % Jacobian of measurement model w.r.t states
    H_p = [0, 0, 1/R(3,3)];                                  % d(h)/d(p)
    H_v = zeros(1,3);                                        % d(h)/d(v)
    H_q = (p(3)/R(3,3)^2).*[-2*q(1) 2*q(2) 2*q(3) -2*q(4)];  % d(h)/d(q)

    H = [H_p H_v H_q];                                       % d(h)/d(x)
    
    % Jacobian of measurement model w.r.t error states
    X_dx = Qmat(q);
    X_dx = blkdiag(eye(6), X_dx);
    H = H*X_dx;                                              %d(h)/d(dx)
        
    % compute innovation and covariance
    z = y(7) - h;
    Z = H*P*H.' + N;
    % compute gain
    K = (P*H.')/Z;
    % compute errors
    dx = K*z;
    % inject errors
    x = injectErrors(x,dx);
    % update P
    P = (eye(9) - K*H)*P*(eye(9) - K*H).' + K*N*K.';
    
end

