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
%   Euler angles to rotation matrix.
%
%   [R,J_e]=e2R(e)
%
%   Gives the rotation matrix body-to-world corresponding to the
%   body orientation given by the Euler angles vector E = [roll; pitch;
%   yaw]; and its Jacobian wrt e if requested.
%
%   Inputs:
%       - e:    Euler angles: e = [roll pitch yaw]'.
%
%   Ouputs:
%       - R:    Rotation matrix body-to-world.
%       - J_e:  Jacobian wrt Euler angles.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [R,J_e]=e2R(e)

r  = e(1); %roll
p  = e(2); %pitch
y  = e(3); %yaw

sr = sin(r);
cr = cos(r);
sp = sin(p);
cp = cos(p);
sy = sin(y);
cy = cos(y);

R= [cp*cy -cr*sy+sr*sp*cy  sr*sy+cr*sp*cy
    cp*sy  cr*cy+sr*sp*sy -sr*cy+cr*sp*sy
    -sp          sr*cp           cr*cp   ];


if nargout > 1

    J_e = [...
        [               0,    -sp*cy,          -cp*sy]
        [               0,    -sp*sy,           cp*cy]
        [               0,       -cp,               0]
        [  sr*sy+cr*sp*cy,  sr*cp*cy, -cr*cy-sr*sp*sy]
        [ -sr*cy+cr*sp*sy,  sr*cp*sy, -cr*sy+sr*sp*cy]
        [           cr*cp,    -sr*sp,               0]
        [  cr*sy-sr*sp*cy,  cr*cp*cy,  sr*cy-cr*sp*sy]
        [ -cr*cy-sr*sp*sy,  cr*cp*sy,  sr*sy+cr*sp*cy]
        [          -sr*cp,    -cr*sp,               0]];

end

return

%% Check with symbolic library (run section)

syms r p y real

e = [r;p;y];

R = e2R(e);

J_e = simplify(jacobian(R,e))
