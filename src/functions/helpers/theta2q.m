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
%   _____________________
% 
%   Theta vector (minimal representation) to quaternion.
%
%   [q]=theta2q(theta)
%
%   Returns the quaternion representing the rotation rates in minimal 
%   representation theta.
%
%   Inputs:
%       - theta:    3x1 Rotation rates vector.
%
%   Ouputs:
%       - q:        Quaternion with convenion: 
%                   q = [qw qx qy qz] with qw the scalar element.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [q]=theta2q(theta)

q=[1;theta/2];

return