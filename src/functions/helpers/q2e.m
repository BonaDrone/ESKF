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
%   Quaternion to Euler angles conversion
%
%   [e,J_q]=q2e(q)
%
%   Returns the Euler angles representing the specified quaternion and 
%   its Jacobian wrt q if requested.
%
%   Inputs:
%       - q:    Quaternion with convenion: 
%               q = [qw qx qy qz] with qw the scalar element.
%
%   Ouputs:
%       - e:    Euler angles: e = [roll pitch yaw]'
%       - J_q:  Jacobian wrt quaternion.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [e,J_q]=q2e(q)

a  = q(1);
b  = q(2);
c  = q(3);
d  = q(4);

y1 =  2*c*d + 2*a*b;
x1 =  a^2   - b^2   - c^2 + d^2;
z2 = -2*b*d + 2*a*c;
y3 =  2*b*c + 2*a*d;
x3 =  a^2   + b^2   - c^2 - d^2;

w  = whos('q');

if strcmp(w.class,'sym')
    
    e = [ atan(y1/x1)
          asin(z2)
          atan(y3/x3) ];
      
else
    
    e = [ atan2(y1,x1)
          asin(z2)
          atan2(y3,x3) ];
      
end

if nargout >1 
    
    dx1dq  = [ 2*a, -2*b, -2*c,  2*d];
    dy1dq  = [ 2*b,  2*a,  2*d,  2*c];
    dz2dq  = [ 2*c, -2*d,  2*a, -2*b];
    dx3dq  = [ 2*a,  2*b, -2*c, -2*d];
    dy3dq  = [ 2*d,  2*c,  2*b,  2*a];
    
    de1dx1 = -y1/(x1^2 + y1^2);
    de1dy1 =  x1/(x1^2 + y1^2);
    de2dz2 =   1/sqrt(1-z2^2);
    de3dx3 = -y3/(x3^2 + y3^2);
    de3dy3 =  x3/(x3^2 + y3^2);
    
    de1dq  = de1dx1*dx1dq + de1dy1*dy1dq;
    de2dq  = de2dz2*dz2dq;
    de3dq  = de3dx3*dx3dq + de3dy3*dy3dq;
    
    J_q     = [de1dq;de2dq;de3dq];
end

return

%% Check with symbolic library (run section; If 0's is correct)

syms a b c d real
q=[a;b;c;d];
[e,Eq] = q2e(q);
simplify(Eq-jacobian(e,q))
