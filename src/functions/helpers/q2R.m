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
%   Quaternion to rotation matrix
%
%   [R,J_R]=q2R(q)
%
%   Returns the rotation matrix from a quaternion (in the specified 
%   convenion) and its rotation Jacobian wrt q. if requested.
%
%   Inputs:
%       - q:    Quaternion with convenion: 
%               q = [qw qx qy qz] with qw th scalar element.
%
%   Ouputs:
%       - R:    Rotation matrix.
%       - J_R:  Rotation matrix Jacobian wrt q.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [R,J_R]=q2R(q)

[a,b,c,d] = split(q);

aa = a^2;
ab = 2*a*b;
ac = 2*a*c;
ad = 2*a*d;
bb = b^2;
bc = 2*b*c;
bd = 2*b*d;
cc = c^2;
cd = 2*c*d;
dd = d^2;

R  = [  aa+bb-cc-dd    bc-ad          bd+ac
        bc+ad          aa-bb+cc-dd    cd-ab
        bd-ac          cd+ab          aa-bb-cc+dd];

if nargout > 1
    
    [a2,b2,c2,d2] = deal(2*a,2*b,2*c,2*d);
    
    J_R = [...
        [  a2,  b2, -c2, -d2]
        [  d2,  c2,  b2,  a2]
        [ -c2,  d2, -a2,  b2]
        [ -d2,  c2,  b2, -a2]
        [  a2, -b2,  c2, -d2]
        [  b2,  a2,  d2,  c2]
        [  c2,  d2,  a2,  b2]
        [ -b2, -a2,  d2,  c2]
        [  a2, -b2, -c2,  d2]];
    
end

return

%% Check with symbolic library (run section)

syms a b c d

q=[a;b;c;d];

aa = a^2;
ab = 2*a*b;
ac = 2*a*c;
ad = 2*a*d;
bb = b^2;
bc = 2*b*c;
bd = 2*b*d;
cc = c^2;
cd = 2*c*d;
dd = d^2;

R  = [  aa+bb-cc-dd    bc-ad          bd+ac
        bc+ad          aa-bb+cc-dd    cd-ab
        bd-ac          cd+ab          aa-bb-cc+dd];


J_qc = simplify(jacobian(R,q))

