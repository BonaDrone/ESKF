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
%   Rotation vector to rotation axis and angle conversion.
%
%   [angle,axis]=vec2aaxis(vec)
%
%   Converts the rotation vector "vec" into its equivalent axis unity
%   vector "axis" and the rotated angle "angle".
%
%   Inputs:
%       - vec:  3x1 Rotation vector.
%
%   Ouputs:
%       - angle:    Rotated angle (rad).
%       - axis:     Axis unity vector.  
%       - Jangle_v: Jacobian of "angle" wrt the vector.
%       - Jaxis_v:  Jacobian of the "axis" wrt the vector.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [angle,axis,Jangle_v,Jaxis_v]=vec2aaxis(vec)

vec = vec(:);
angle = sqrt(dot(vec,vec));

s = whos('angle');

if (strcmp(s.class,'sym')) || (angle>eps)
    axis = vec/angle;
    if nargout > 2
        Jangle_v = axis';
        if nargout > 3
            Jaxis_v = [...
            [ 1/angle-axis(1)^2/angle,  -axis(1)/angle*axis(2), -axis(1)/angle*axis(3) ]
            [ -axis(1)/angle*axis(2),  1/angle-axis(2)^2/angle, -axis(2)/angle*axis(3) ]
            [ -axis(1)/angle*axis(3),  -axis(2)/angle*axis(3), 1/angle-axis(3)^2/angle ]];
        end
    end

else
    angle  = 0;
    axis  = [0;0;0];
    if nargout > 2
        warning('TODO: revise Jacobian')
        Jangle_v = [0 0 0];
        if nargout > 3
            Jaxis_v = zeros(3);
        end
    end
end

return

%% Check with symbolic library (run section. If 0's, correct.)

syms u v w real
vec = [u;v;w];
[angle,axis,Jangle_v,Jaxis_v] = vec2aaxis(vec)

Jangle_v - jacobian(angle,rv)
Jaxis_v - jacobian(axis,rv)
