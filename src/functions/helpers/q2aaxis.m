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
%   Quaternion to rotated angle and rotation axis vector.
%
%   [angle,axis] = q2aaxis()
%
%   Returns the rotation of "angle" rad around the axis defined by 
%   the unity vector "axis", that is equivalent to that defined by 
%   the quaternion q.
%
%   Inputs:
%       - q:    Quaternion with convenion: 
%               q = [qw qx qy qz] with qw th scalar element.
%
%   Ouputs:
%       - angle:    Rotation angle (rad).
%       - axis:     Rotation axis (unity vector).
%       - Jangle_q: Angle Jacobian wrt q.
%       - Jaxis_q:  Axis Jacobian wrt q.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [angle,axis,Jangle_q,Jaxis_q] = q2aaxis(q)

axis = q(2:4);
axis = axis/norm(axis);

angle = 2*acos(q(1));

if nargout > 2
    Jangle_q = [-2/sqrt(1 - q(1)^2) 0 0 0];
end
if nargout > 3
    den = (abs(q(2))^2 + abs(q(3))^2 + abs(q(4))^2)^(3/2);
    s = abs(q(2))^2 + abs(q(3))^2 + abs(q(4))^2;
    Jaxis_q = [ 0   (s - q(2)*abs(q(2))*sign(q(2)))/den      -(q(2)*abs(q(3))*sign(q(3)))/den           -(q(2)*abs(q(4))*sign(q(4)))/den;
                0   -(q(3)*abs(q(2))*sign(q(2)))/den          (s - q(3)*abs(q(3))*sign(q(3)))/den       -(q(3)*abs(q(4))*sign(q(4)))/den;
                0   -(q(4)*abs(q(2))*sign(q(2)))/den         -(q(4)*abs(q(3))*sign(q(3)))/den           (s - q(4)*abs(q(4))*sign(q(4)))/den];
end

return

%% Check with symbolic library (run section)

syms w x y z

q=[w;x;y;z];

axis = q(2:4);
axis = axis/norm(axis);

angle = 2*acos(q(1));

Jangle_q = simplify(jacobian(angle,q))
Jaxis_q = simplify(jacobian(axis,q)) 
