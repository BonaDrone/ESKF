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
%   Quaternion normalization to unit length
%
%   [q] = q_int_euler(ts,w_pre,w_pos)
%
%   Returns the unit length normalized quaternion and its Jacobian 
%   wrt q. Note that this Jacobian is a symmetric 4x4 matrix.
%   
%   Inputs:
%       - q:        Quaternion with convention:
%                   q = [qw qx qy qz] with qw the scalar element.
% 
%   Outputs:
%       - qn:       Normalized quaternion with convention:
%                   qn = [qw qx qy qz] with qw the scalar element.
%       - Q_q:      Jacobian wrt q.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [qn,Q_q]=qNorm(q)


nq = sqrt(q(:)'*q(:));
qn = q/nq;


if nargout > 1
    
    a = q(1);
    b = q(2);
    c = q(3);
    d = q(4);

    nq3 = nq^3;
    
    Q_q = [...
        [ (b^2+c^2+d^2)/nq3,          -a/nq3*b,          -a/nq3*c,          -a/nq3*d]
        [          -a/nq3*b, (a^2+c^2+d^2)/nq3,          -b/nq3*c,          -b/nq3*d]
        [          -a/nq3*c,          -b/nq3*c, (a^2+b^2+d^2)/nq3,          -c/nq3*d]
        [          -a/nq3*d,          -b/nq3*d,          -c/nq3*d, (a^2+b^2+c^2)/nq3]];
end
return

%% Check with symbolic library (run section; If 0's is correct)

syms a b c d real
q = [a;b;c;d];
[qn,QNq] = qNorm(q);

QNq - simple(jacobian(qn,q))

QNq - QNq'