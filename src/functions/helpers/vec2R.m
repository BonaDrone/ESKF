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
%   Rotation vector to rotation matrix.
%
%   [R] = vec2R(vec)
%
%   Computes the rotation matrix corresponding to the rotation vector vec.
%   Uses rodrigues formula.
%
%   Inputs:
%       - vec:  3x1 Rotation vector.
%
%   Ouputs:
%       - R:    Rotation matrix.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [R] = vec2R(vec)

[angle,axis] = vec2aaxis(vec); % u is always a column vector, regardless of the charactrer of v.

% intermediate results
ca  = cos(angle);
sau = sin(angle)*axis;

% R = cos(a)*eye(3) + sin(a)*hat(u) + (1-cos(a))*u*u'; A shortcut is:
R = diag([ca;ca;ca]) + vec2skew(sau) + ((1-ca)*axis)*axis';

return