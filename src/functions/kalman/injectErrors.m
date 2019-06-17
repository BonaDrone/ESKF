function [x] = injectErrors(x,dx)
%INJECTERRORS inject the computed state errors in the state
%   Detailed explanation goes here

    % Inject position and velocity errors
    x(1) = x(1) + dx(1); % position
    x(2) = x(2) + dx(2);
    x(3) = x(3) + dx(3);
    x(4) = x(4) + dx(4); % velocity
    x(5) = x(5) + dx(5);
    x(6) = x(6) + dx(6);
    x(7) = x(7) + dx(7); % ab
    x(8) = x(8) + dx(8);
    x(9) = x(9) + dx(9);
    
    % Inject orientation error
%     tmp = [1; dx(7)/2; dx(8)/2; dx(9)/2];
% 
%     q_aux = qProd(x(7:10), tmp);
%     q_aux = q_aux/norm(q_aux);
%     
%     x(7:10) = q_aux;

end

