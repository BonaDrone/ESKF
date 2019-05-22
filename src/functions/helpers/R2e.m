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
%   Rotation to Euler angles.
%
%   [R,J_e]=e2R(e)
%
%   Gives the Euler angles corresponding to the rotation matrix 
%   body-to-world.
%
%   Inputs:
%       - R:    Rotation matrix body-to-world.
%
%   Ouputs:
%       - e:    Euler angles: e = [roll pitch yaw]'.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [e]=R2e(R)

s = whos('R');

if (strcmp(s.class,'sym'))
    roll  = atan(R(3,2)/R(3,3));
    pitch = asin(-R(3,1));
    yaw   = atan(R(2,1)/R(1,1));
else
    roll  = atan2(R(3,2),R(3,3));
    pitch = asin(-R(3,1));
    yaw   = atan2(R(2,1),R(1,1));
end

e = [roll;pitch;yaw];



