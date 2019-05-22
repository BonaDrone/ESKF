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
%   QROT Vector rotation via quaternion algebra.
% 
%   W = QROT(V,Q) performs to vector V the rotation specified by
%   quaternion Q.
%
%   [w, W_v, W_q] = QROT(V,Q) returns Jacobians wrt V and Q. Note that
%   this only works with single points V = [x;y;z].
%
%   QROT is equivalent to Rp, with the exception that the arguments come in
%   different order.
%
%   See also QUATERNION, Q2Q, QPROD, RP, RTP.
% 
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [w, W_v, W_q] = qRot(v,q)

v0 = [0;v];

w = qProd(qProd(q,v0),q2qc(q));

w = w(2:end);

if nargout > 1 % we want Jacobians

    if size(v,2) == 1 % Jacobians are computed exactly as in function Rp.m.

        [a,b,c,d] = split(q);
        [x,y,z]   = split(v);

        axdycz = 2*(a*x - d*y + c*z);
        bxcydz = 2*(b*x + c*y + d*z);
        cxbyaz = 2*(c*x - b*y - a*z);
        dxaybz = 2*(d*x + a*y - b*z);

        W_q = [...
            [  axdycz,  bxcydz, -cxbyaz, -dxaybz]
            [  dxaybz,  cxbyaz,  bxcydz,  axdycz]
            [ -cxbyaz,  dxaybz, -axdycz,  bxcydz]];

        W_v = q2R(q);
        
    else
        error('Jacobians only available for single points')
    end
end

return

%% BUILD AND TEST JACOBIANS

syms a b c d x y z real

q = [a;b;c;d];
p = [x;y;z];

[rp1,RPq1,RPp1] = Rp(q,p);
[rp,RPp,RPq]    = qRot(p,q);

RPq1 = simple(jacobian(rp,q));
RPp1 = simple(jacobian(rp,p));

ERPq = simplify(RPq-RPq1)
ERPp = simplify(RPp-RPp1)
