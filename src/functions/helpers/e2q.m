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
%   Euler angles to quaternion conversion
%
%   [q,J_e]=e2q(e)
%
%   Returns the quaternion representing the specified Euler angles and 
%   its Jacobian wrt the Euler angles if requested.
%
%   Inputs:
%       - e:    Euler angles: e = [roll pitch yaw]'
%
%   Ouputs:
%       - q:    Quaternion with convenion: 
%               q = [qw qx qy qz] with qw the scalar element.
%       - J_e:  Jacobian wrt Euler anglea.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [q,J_e]=e2q(e)

qx = aaxis2q(e(1),[1;0;0]); %  roll rot. on X axis
qy = aaxis2q(e(2),[0;1;0]); % pitch rot. on Y axis
qz = aaxis2q(e(3),[0;0;1]); %   yaw rot. on Z axis

q = qProd(qProd(qz,qy),qx);

if nargout == 2

    sr = sin(e(1)/2);
    sp = sin(e(2)/2);
    sy = sin(e(3)/2);

    cr = cos(e(1)/2);
    cp = cos(e(2)/2);
    cy = cos(e(3)/2);

    J_e = 0.5*[
        [ -cy*cp*sr+sy*sp*cr, -cy*sp*cr+sy*cp*sr, -sy*cp*cr+cy*sp*sr]
        [  cy*cp*cr+sy*sp*sr, -cy*sp*sr-sy*cp*cr, -sy*cp*sr-cy*sp*cr]
        [ -cy*sp*sr+sy*cp*cr,  cy*cp*cr-sy*sp*sr, -sy*sp*cr+cy*cp*sr]
        [ -sy*cp*sr-cy*sp*cr, -cy*cp*sr-sy*sp*cr,  cy*cp*cr+sy*sp*sr]
        ];
end

return

%% Check with symbolic library (run section; If 0's is correct)

syms r p y real             % Declare symbolic real variables
e = [r;p;y];                % build Euler vector

[q,Qe] = e2q(e);            % Call function to test with symbolic input

simplify(Qe-jacobian(q,e))  % Verify that jacobian() returns the same as our Jacobian.






