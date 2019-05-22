% Copyright (C) by A. Santamaria-Navarro and Joan Sola (asantamaria@iri.upc.edu, jsola@iri.upc.edu) 
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
%   Euler method
%
%   [q] = q_int_euler(ts,w_pre,w_pos)
%
%   Returns the quaternion corresponding to the euler
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
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [q] = q_int_euler(ts,w_pre,w_pos)

q0=[1;0;0;0];
q1=0.5*qProd([1;0;0;0],[0;w_pre]);
q2=0.5*qProd(q0+q1,[0;w_pos]);
q=qNorm(q0+(q1+q2)*ts/2);  

return