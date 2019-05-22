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
%   Time update function for quaternions.
%
%   [q1,Q1_q,Q1_w] = qPredict(q,w,dt,met)
%
%   Returns the quaternion after a rotation in body frame expressed 
%   by the three angles in DV (roll, pitch, yaw).
%
%   Qu = QPREDICT(Q,DV) is the updated quaternion Q after a rotation 
%   in body frame expressed by the three angles in DV (roll, pitch, yaw).
%
%   Qu = QPREDICT(Q,W,DT) assumes a rotation speed W and a 
%   sampling time DT. It is equivalent to the previous case with DV = W*DT.
%
%   [Qu,QU_q,QU_w] = QPREDICT(Q,W,DT) returns Jacobians wrt Q and W.
%
%   [...] = QPREDICT(...,MET) allows the specification of the method to
%   update the quaternion:
%       'zerof' Zeroth order forward using Qu = QPROD(Q,V2Q(W(k-1)*DT))
%       'zerob' Zeroth order backward using Qu = QPROD(Q,V2Q(W(k)*DT))
%       'first' First order using Qu = QPROD(Q,W2) with W2 the Taylor
%       expansion of the derivative and the average rate during dt.
%   The Jacobians are always computed according to the 'Euler' method, 
%   which is the approximation of the zeroth order integration.
%   'Zerof' is the default method.
%   
%   Inputs:
%       - q:    Quaternion.
%       - w:    Rotation in body frame or Rotation speed expressed 
%               by the three angles (roll, pitch, yaw) or three angle rates.
%       - dt:   Time step..
%       - met:  Update method specification.
% 
%   Outputs:
%       - q1:   Quaternion after a rotation in body frame.
%       - Q_q:  Jacobian wrt quaternion.
%       - Q_w:  Jacobian wrt rotation angles or angular rates.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function [q1,Q_q,Q_w]=qPredict(q,w,dt,met)


switch nargin
    case 2
        dt = 1;
        met = 'zerof';
    case 3
        if isa(dt,'char')
            met = dt;
            dt = 1;
        else
            met = 'zerof';
        end
end


switch lower(met)
    case 'zerof'
        q1 = qNorm(qProd(q,vec2q(w(:,2)*dt))); 
    case 'zerob'
        q1 = qNorm(qProd(q,vec2q(w(:,1)*dt))); 
    case 'first'
        wdot = (w(:,2)-w(:,1))/dt;
        w_avg = w(:,1)+0.5*wdot*dt;
        q_avg = vec2q(w_avg*dt);
        q2 = qProd(0.0417*dt^2*q,[0;cross(w(:,1),w(:,2))]);
        dq = q_avg+q2;
        q1 = qNorm(qProd(q,dq));
    otherwise
        error('Unknown quaternion predict method. Use ''zerof'', ''zerob'' or ''first''')
end

if nargout > 1 
    % Jacobians based on Euler form which corresponds to:
        %W  = w2omega(w);
        %q1 = qNorm(q + .5*dt*W*q);       
    W    = w2omega(w);
    Q_q = eye(4) + 0.5*dt*W;
    Q_w = 0.5*dt*q2Pi(q);
end
