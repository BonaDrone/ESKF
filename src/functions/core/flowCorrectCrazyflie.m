function [x,P] = flowCorrectCrazyflie(x,P,y,dt)
%FLOWCORRECTCRAZYFLIE Correct state estimation with flow data
%   

  persistent n; n = 0.0625;
  persistent N; N = [n, 0; 0, n];

  % Saturate elevation in prediction and correction to avoid singularities
  if ( x(3) < 0.1 )
      height = 0.1;
  else
      height = x(3);
  end
  
  R = q2R([x(7), x(8), x(9), x(10)]);
  
  Npix = 30.0; % [pixels] (same in x and y)
  thetapix = (pi/180) * 4.2; % [rad] (same in x and y)
  omegaFactor = 1.25;
  
  % predicted number of accumulated pixels
  predictedNX = (dt * Npix / thetapix ) * ((x(4) * R(3,3) / height) - omegaFactor * y(5));
  predictedNY = (dt * Npix / thetapix ) * ((x(5) * R(3,3) / height) + omegaFactor * y(4));
  % measured number of accumulated pixels
  measuredNX = y(8);
  measuredNY = y(9);
  
  % Derive x measurement equation with respect to the error states (effectively vx and z)
  h_zx = (Npix * dt / thetapix) * ((R(3,3) * x(4)) / (-height * height));
  h_vx = (Npix * dt / thetapix) * (R(3,3) / height);
  
  % Derive y measurement equation with respect to the error states (effectively vy and z)
  h_zy = (Npix * dt / thetapix) * ((R(3,3) * x(5)) / (-height * height));
  h_vy = (Npix * dt / thetapix) * (R(3,3) / height);
  
  % Jacobian of measurement model with respect to the error states
  % dh/dx * dx/ddx where dx/ddx = blkdiag(I_6 Q)
  H = [ 0 0 h_zx h_vx  0   0 0 0 0;...
        0 0 h_zy  0   h_vy 0 0 0 0;];
  
  % compute innovation and covariance
  z = [measuredNX - predictedNX; measuredNY - predictedNY];
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

