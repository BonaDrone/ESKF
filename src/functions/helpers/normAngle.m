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
%   NORMANGLE Normalize angle to (-pi .. pi] values.
%
%   ao = normAngle(ai,deg)
%
%   NORMANGLE(A) brings angle A to the range (-pi .. pi] by adding an
%   integer number of turns.
%   
%   NORMANGLE(A,DEG) with DEG ~= false, uses degrees instead of radians,
%   and thus the angle A is normalized to the range (-180 .. 180].
%
%   Inputs:
%       - ai:   Angle to be normalized.
%       - deg:  Flag to choose degrees instead of radians.
%   Ouputs:
%       - ao:   Normalized angle.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function ao = normAngle(ai,deg)

if nargin < 2
    deg = false;
end

if deg
    halfTurn = 180;
else
    halfTurn = pi;
end

ao = ai;
while any(ao <= -halfTurn)
    ao(ao <= -halfTurn) = ao(ao <= -halfTurn) + 2*halfTurn;
end
while any(ao > halfTurn)
    ao(ao > halfTurn) = ao(ao > halfTurn) - 2*halfTurn;
end