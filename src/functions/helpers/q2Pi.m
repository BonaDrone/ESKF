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
%   Q2PI  Pi matrix construction from quaternion.
% 
%   PI = Q2PI(Q) Jacobian submatrix PI from quaternion
%
%   Given:  Q     = [a b c d]'  the attitude quaternion
%           W     = [p q r]'    the angular rates vector
%           OMEGA = W2OMEGA(W)  a skew symetric matrix 
%
%   The output matrix:
%
%                |-b -c -d |
%           PI = | a -d  c |  
%                | d  a -b |
%                |-c  b  a |  
% 
%   is the Jacobian of OMEGA*Q with respect to W
%
%   See also W2OMEGA, Q2R
%     
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function Pi = q2Pi(q)

Pi = [-q(2) -q(3) -q(4)
       q(1) -q(4)  q(3)
       q(4)  q(1) -q(2)
      -q(3)  q(2)  q(1)];
  
return

