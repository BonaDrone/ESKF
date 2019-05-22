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
% You should have r maeceived a copy of the GNU Leser General Public License
% along with QuadOdom.  If not, see <http://www.gnu.org/licenses/>.
% 
%   __________________________________________________________________
% 
%   Quaternion conjugate
%
%   [q_c,J_qc]=q2qc(q)
%
%   Returns the quaternion conjugate (in the specified convenion) and 
%   its Jacobian wrt q if requested.
%
%   Inputs:
%       - q:    Quaternion with convenion: 
%               q = [qw qx qy qz] with qw the scalar element.
%
%   Ouputs:
%       - q_c:  Quaternion conjugate maintaining the input convenion.
%       - J_qc: Jacobian of quaternion conjugate.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [q_c,J_qc]=q2qc(q)

q_c = q;
q_c(2:end) = -q_c(2:end);

if nargout > 1
    J_qc = diag([1 -1 -1 -1]);
end

return

%% Check with symbolic library (run section)

syms w x y z

q=[w;-x;-y;-z];

J_qc = simplify(jacobian(q))
