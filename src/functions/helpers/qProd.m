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
%   Quaternion product
%
%   [q,Qq1,Qq2] = qProd(q1,q2)
%
%   Returns the quaternion product q1*q2.
%
%   Inputs:
%       - q1:   Quaternion with convenion: 
%               q1 = [qw qx qy qz] with qw the scalar element.
%       - q2:   Quaternion with convenion: 
%               q2 = [qw qx qy qz] with qw the scalar element.
%
%   Ouputs:
%       - q:    Quaternion with convenion: 
%               q = [qw qx qy qz] with qw the scalar element.
%       - J_q1: Jacobian wrt q1 (if requested).
%       - J_q2: Jacobian wrt q2 (if requested).
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [q,J_q1,J_q2] = qProd(q1,q2)

% [a,b,c,d] = split(q1);
a = q1(1);
b = q1(2);
c = q1(3);
d = q1(4);
% [w,x,y,z] = split(q2);
w = q2(1);
x = q2(2);
y = q2(3);
z = q2(4);

q = [...
    a*w - b*x - c*y - d*z
    a*x + b*w + c*z - d*y
    a*y - b*z + c*w + d*x
    a*z + b*y - c*x + d*w];

if nargout > 1
    J_q1 = [...
        [  w, -x, -y, -z]
        [  x,  w,  z, -y]
        [  y, -z,  w,  x]
        [  z,  y, -x,  w]];

    J_q2 = [...
        [  a, -b, -c, -d]
        [  b,  a, -d,  c]
        [  c,  d,  a, -b]
        [  d, -c,  b,  a]];
end

return

%% Check with symbolic library (Run section. Result 0 means correct)

syms a b c d w x y z
q1=[a b c d]';
q2 = [w x y z]';

[q,Qq1,Qq2] = qProd(q1,q2);

simplify(Qq1 - jacobian(q,q1))
simplify(Qq2 - jacobian(q,q2))
