function [x,P] = accelCorrect(x,P,y)
%ACCELCORRECT Correct state estimation with accelerometer data
%

    persistent n; n = 2.0;
    persistent N; N = [n, 0, 0; 0, n, 0; 0, 0, n];
    persistent g; g = 9.80665;
    
    
    H = [ 0 0 0 0 0 0                    0                      -x(7)*x(7)+x(8)*x(8)+x(9)*x(9)-x(10)*x(10)*-g 2*x(7)*x(8)+2*x(9)*x(10)*-g;...
          0 0 0 0 0 0 x(7)*x(7)-x(8)*x(8)-x(9)*x(9)+x(10)*x(10)*-g                     0                      2*x(7)*x(9)-2*x(8)*x(10)*-g;...
          0 0 0 0 0 0        -2*x(7)*x(8)-2*x(9)*x(10)*-g                  2*x(8)*x(10)-2*x(7)*x(9)*-g                        0];
                

    h = [ (2*x(7)*x(9) - 2*x(8)*x(10)) * -g;...
          (-2*x(7)*x(8) - 2*x(9)*x(10)) * -g;...
          (-x(7)*x(7) + x(8)*x(8) + x(9)*x(9) - x(10)*x(10)) * -g];
                               
    y_norm = y(1:3)/norm(y(1:3));  
    h_norm = h/norm(h);  
    
%     % measurement model
%     h = [ g*(2*x(7)*x(9) - 2*x(8)*x(10));
%          -g*(2*x(7)*x(8) + 2*x(9)*x(10));
%          -g*(x(7)^2 - x(8)^2 - x(9)^2 + x(10)^2)];
%     
%     % Jacobian of measurement model w.r.t. error states
%     H = [ 0, 0, 0, 0, 0, 0,                                 0, - g*x(7)^2 + g*x(8)^2 + g*x(9)^2 - g*x(10)^2, 2*g*x(7)*x(8) + 2*g*x(9)*x(10);...
%           0, 0, 0, 0, 0, 0, g*x(7)^2 - g*x(8)^2 - g*x(9)^2 + g*x(10)^2,                                   0, 2*g*x(7)*x(9) - 2*g*x(8)*x(10);...
%           0, 0, 0, 0, 0, 0,           - 2*g*x(7)*x(8) - 2*g*x(9)*x(10),               2*g*x(8)*x(10) - 2*g*x(7)*x(9),                     0];

    % compute innovation and covariance
    z = y_norm - h_norm;
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

