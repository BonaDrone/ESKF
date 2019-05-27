function [x] = injectErrors(x,dx)
%INJECTERRORS Summary of this function goes here
%   Detailed explanation goes here

    % Inject position and velocity errors
    x(1) = x(1) + dx(1); % position
    x(2) = x(2) + dx(2);
    x(3) = x(3) + dx(3);
    x(4) = x(4) + dx(4); % velocity
    x(5) = x(5) + dx(5);
    x(6) = x(6) + dx(6);
    
    % Inject orientation error
    tmp = [1; dx(7)/2; dx(8)/2; dx(9)/2];
    
    qL = [ x(7)  -x(8)  -x(9)  -x(10);...
           x(8)   x(7)  -x(10)  x(9);...
           x(9)   x(10)  x(7)  -x(8);...
           x(10) -x(9)   x(8)   x(7)];

    tmp2 = (qL * tmp)/norm(qL * tmp);             
          
    x(7) = tmp2(1);
    x(8) = tmp2(2);
    x(9) = tmp2(3);
    x(10) = tmp2(4);

end

