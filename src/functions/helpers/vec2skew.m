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
%   3x1 Vector to Skew-symmetric matrix transformation
%
%   [M_sk,J_sk]=vec2skew(vec)
%
%   Returns the skew-symmetric matrix and the corresponding Jacobian wrt 
%   vec if requested. 
%
%   Inputs:
%       - vec:    3x1 Vector.
%
%   Ouputs:
%       - M_sk: Skew symmetric matrix.
%       - J_sk: Jacobian of the skew-symmetric matrix.  
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [M_sk,J_sk]=vec2skew(vec)


if (size(vec)==[3 1])
    
    %Skew-symmetric matrix
    M_sk=[ 0        -vec(3)   vec(2);
          vec(3)       0     -vec(1);
         -vec(2)     vec(1)      0 ];
     
    %Jacobian of skew-symmetric matrix
    if nargout > 1 
        J_sk=[  0     0       0;
                0     0       1;
                0    -1       0;
                0     0      -1;
                0     0       0;
                1     0       0;
                0     1       0;
               -1     0       0;
                0     0       0];
            
     end
else
    error('Input dimensions don''t agree. Enter 3x1 column vector')
end

return

%% Check with symbolic library (run section)

syms x y z

vec = [x;y;z];

M_sk=[ 0        -vec(3)     vec(2);
      vec(3)       0       -vec(1);
     -vec(2)     vec(1)        0 ];

J_sk = simplify(jacobian(M_sk,vec))