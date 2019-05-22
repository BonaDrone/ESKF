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
%   Angular rates vector w to Omega matrix defined as:
%
%   omega=[  0   -W(1) -W(2) -W(3)
%           W(1)   0    W(3) -W(2)
%           W(2) -W(3)   0    W(1)
%           W(3)  W(2) -W(1)   0  ];
%   
%
%   [Omega] = w2omega(w)
%
%   Returns the matrix Omega from angular rates vector w.
%
%   Inputs:
%       - w:    Angular rates vector.
%
%   Ouputs:
%       - Omega:    Omega matrix.
%       - J_w:      Jacobian of Omega wrt w (if requested).
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [Omega,J_w] = w2omega(w)

if all(size(w) == [3 1])
    Omega=[  0   -w(1) -w(2) -w(3)
            w(1)   0    w(3) -w(2)
            w(2) -w(3)   0    w(1)
            w(3)  w(2) -w(1)   0  ];
else
    error('Input dimensions don''t agree. Enter 3x1 column vector')
end

if nargout > 1
    J_w = [0  0  0;
           1  0  0;
           0  1  0;
           0  0  1;
          -1  0  0;
           0  0  0;
           0  0 -1;
           0  1  0;
           0 -1  0;
           0  0  1;
           0  0  0;
          -1  0  0;
           0  0 -1;
           0 -1  0;
           1  0  0;
           0  0  0]; 
end

return

%% Check with symbolic library (run section)

syms x y z

w=[x;y;z];

[Omega,J_w] = w2omega(w);    

simplify(J_w - jacobian(Omega,w))

