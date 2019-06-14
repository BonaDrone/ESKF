function [x,P] = accelCorrect(x,P,y)
%ACCELCORRECT Correct state estimation with accelerometer data
%

    persistent n; n = 5.0;
    persistent N; N = [n, 0, 0; 0, n, 0; 0, 0, n];
    persistent gz; gz = 9.80665;

    g = [0; 0; -gz];
    
    qw = x(7);
    qx = x(8);
    qy = x(9);
    qz = x(10);
    q = [qw qx qy qz];

    R = q2R(q);

    % measurement model
%     h = -R.' * g;
%     h = [ (2*x(7)*x(9) - 2*x(8)*x(10)) * -gz;...
%           (-2*x(7)*x(8) - 2*x(9)*x(10)) * -gz;...
%           (-x(7)*x(7) + x(8)*x(8) + x(9)*x(9) - x(10)*x(10)) * -gz];
      
    h = -R'*g;
%     % Jacobian of measurement model w.r.t. state
%     H_p = zeros(3,3);                                       % d(h)/d(p)
%     H_v = zeros(3,3);                                       % d(h)/d(v)
%     H_q = [ 2*gz*qy, -2*gz*qz,  2*gz*qw, -2*gz*qx;...       % d(h)/d(q)
%            -2*gz*qx, -2*gz*qw, -2*gz*qz, -2*gz*qy;...
%            -2*gz*qw,  2*gz*qx,  2*gz*qy, -2*gz*qz];                
%        
%     H = [H_p H_v H_q];                                      % d(h)/d(x)
% 
%     % Jacobian of measurement model w.r.t error states
%     X_dx = Qmat(q);
%     X_dx = blkdiag(eye(6), X_dx);
%     H = H*X_dx;                                             %d(h)/d(dx)

%     H = [ 0 0 0 0 0 0                    0                      -x(7)*x(7)+x(8)*x(8)+x(9)*x(9)-x(10)*x(10)*-gz 2*x(7)*x(8)+2*x(9)*x(10)*-gz;...
%           0 0 0 0 0 0 x(7)*x(7)-x(8)*x(8)-x(9)*x(9)+x(10)*x(10)*-gz                    0                      2*x(7)*x(9)-2*x(8)*x(10)*-gz;...
%           0 0 0 0 0 0        -2*x(7)*x(8)-2*x(9)*x(10)*-gz                  2*x(8)*x(10)-2*x(7)*x(9)*-gz                        0];

    H = zeros(3, 6);

    y_norm = y(1:3)/norm(y(1:3));
    h_norm = h/norm(h);

    % compute innovation and covariance
    z = y_norm - h_norm;
    % z = y - h;
    Z = H*P*H.' + N;
    % compute gain
    K = (P*H.')/Z;
    % compute errors
    dx = K*z;
    % inject errors
    x = injectErrors(x,dx);
    % update P
    P = (eye(6) - K*H)*P*(eye(6) - K*H).' + K*N*K.';
      
end

