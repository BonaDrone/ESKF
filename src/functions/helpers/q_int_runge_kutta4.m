% Copyright (C) by A. Santamaria-Navarro (asantamaria@iri.upc.edu)
%
% This file is part of MATLAB QuadOdom. You can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% QuadOdom is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
%
% You should have received a copy of the GNU Leser General Public License
% along with QuadOdom.  If not, see <http://www.gnu.org/licenses/>.
% 
%   __________________________________________________________________
% 
%   Quaternion integration using angular velocity readings and computing
%   Runge-Kutta 4th order method
%
%   [q] = q_int_runge_kutta4(ts,w_pre,w_pos)
%
%   Returns the quaternion corresponding to the 4th order Runge-Kutta
%   integration of the ODE equation
%   
%   q_dot = 0.5*q*[0;wx;wy;wz]
%   
%   Inputs:
%       - ts:       Time step.
%       - w_pre:    Angular velocity at time k-1.
%       - w_pos:    Angular velocity at time k.
% 
%   Outputs:
%       - q:        Relative quaternion produced by the angular velocity 
%                   with convention:
%                   q = [qw qx qy qz] with qw the scalar element.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   _____________________

function [q] = q_int_runge_kutta4(ts,w_pre,w_pos)

q0=[1;0;0;0];
d_mean=(w_pre+w_pos)/2;
k1 = 0.5*qProd(q0,[0;w_pre]);
% q1 = qnorm(q0+(ts/2*k1));
q1 = (ts/2)*k1;
k2 = 0.5*qProd(q0+q1,[0;d_mean]);
% q2 = qnorm(q0+(ts/2*k2));
q2 = (ts/2)*k2;
k3 = 0.5*qProd(q0+q2,[0;d_mean]);
% q3 = qnorm(q0+(ts*k3));
q3 = ts*k3;
k4 = 0.5*qProd(q0+q3,[0;w_pos]);
% q = qProd(qnorm(ts/6 * (k1+2*k2+2*k3+k4)),q0); 
q = q0 + ts/6*(k1+2*k2+2*k3+k4); 

return