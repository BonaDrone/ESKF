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
%   Rotaiton vector to quaternion conversion.
%
%   [q,Qv] = vec2q(v)
%
%   Returns the quaternion Q correscponding to the rotation
%   encoded in rotation vector V, and the associated Jacobian Qv = dQ/dV.
%   
%   Inputs:
%       - v:    Rotation vector.
%
%   Ouputs:
%       - q:    Quaternion with convenion: 
%               q = [qw qx qy qz] with qw the scalar element.
%       - J_v:  Jacobian wrt rotation vector.    
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [q,J_v] = vec2q(v)

if nargout == 1
    
    [a,u] = vec2aaxis(v);
    q = aaxis2q(a,u);

else
    a = sqrt(dot(v,v));

    if isnumeric(a) && a < 1e-6
        
        % Use small signal approximation:
        q = [1-norm(v)^2/8
            v(:)/2];
        
        J_v = [-1/4*v(:)'
            0.5*eye(3)];

    else

        [a,u,Av,Uv] = vec2aaxis(v);
        [q,Qa,Qu] = aaxis2q(a,u);
        J_v = Qa*Av + Qu*Uv;

    end
end


return

%% Check Jacobian with symbolic library
syms r s t real
v = [r;s;t];
[q,Qv] = vec2q(v)

simplify(Qv - jacobian(q,v))