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
%   Rotated angle and rotation axis vector to quaternion.
%
%   [q] = aaxis2q(angle,axis)
%
%   Returns the quaternion representing a rotation of "angle" rad 
%   around the axis defined by the unit vector "axis".
%
%   Inputs:
%       - angle:    Rotation angle (rad).
%       - axis:     Rotation axis (unity vector).
%
%   Ouputs:
%       - q:        Quaternion with convenion: 
%                   q = [qw qx qy qz] with qw the scalar element.
%       - J_angle:  Jacobian wrt "angle".
%       - J_axis:   Jacobian wrt "axis".
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [q,J_angle,J_axis] = aaxis2q(angle,axis)

q = [  cos(angle/2)
    axis*sin(angle/2)];

if nargout > 1
    J_angle = [ -sin(angle/2)/2
                axis*cos(angle/2)/2];
end
if nargout > 2
    J_axis = [0 0 0;eye(3)*sin(angle/2)];
end

return

%% Check with symbolic library (run section)

syms angle x y z

axis=[x;y;z];

q = [  cos(angle/2)
    axis*sin(angle/2)];

J_angle = simplify(jacobian(q,angle));
J_axis = simplify(jacobian(q,axis));

return