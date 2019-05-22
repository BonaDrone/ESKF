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
%   Quaternion-based pose to Euler-based pose
% 
%   [ep,EPq] = qpose2epose(qp)
% 
%   returns a full 6-pose EP=[X;E] from a full 7-pose QP=[X;Q] where 
%   X is 3D opsition and Q and E are 3D orientations. Optionally can return
%   Jacobian of the operation.
% 
%   - Inputs:
%       - GTRob:    Ground truth with robot structure (see newrobot.m)
%       - Rob:      Estimation with robot structure (see newrobot.m)
% 
%   - Outputs:
%       - NEES:    Error analysis (rmse, anees, etc.)
%   
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu). 
%   Copyright 2016 asantamaria@iri.upc.edu.
%   __________________________________________________________________

function [ep,EPq] = qpose2epose(qp)

if any(size(qp) ~= [7,1])
    warning('Input pose should be a column 7-vector')
end

ep = zeros(6,1);
ep(1:3) = qp(1:3);
P = eye(3);
[ep(4:6),Eq] = q2e(qp(4:7));
ep(4:6) = normAngle(ep(4:6));
EPq = [P zeros(3,4);zeros(3) Eq];

return