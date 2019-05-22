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
%    ISNORMQ(Q) checks to see if Q is a normalized quaternion or set of quaternions.
%     
%    returns a value accordingly:
%
%        0 if Q is not a normalized quaternion or a vector of normalized
%        quaternions.
%
%        1 if Q is 4xN and only the columns are normalized.
%
%        2 if Q is Nx4 and only the rows are normalized.
%
%        3 if Q is 4x4 and both the columns and rows are normalized.
%
%     The test for normalization uses 2*EPS as a tolerance.
%
%     Some texts refer to a normalized quaternion as a "versor".
%
%     See also ISQ, EPS.
%
%   Copyright 2016 asantamaria@iri.upc.edu.
%   Extracted from slamToolbox (Joan Sola @ iri.upc.edu).
%   __________________________________________________________________

function qtype=isnormq(q)

if nargin~=1
  error('isnormq() requires one input argument');
else
  
  size_q=size(q);

  if ( length(size_q)~=2 || max(size_q==4)~=1 )
    qtype=0; % Not a normalized quaternion or quaternion vector
  else
    
    tol=2*eps;
    
    cols_are_norm = size_q(1)==4 & ~sum((sum(q.^2,1)-ones(1,size_q(2)))>tol);
    rows_are_norm = size_q(2)==4 & ~sum((sum(q.^2,2)-ones(size_q(1),1))>tol);

    if ( ~cols_are_norm && ~rows_are_norm )
      qtype=0; % Not a normalized quaternion or quaternion vector
 
    elseif ( cols_are_norm && ~rows_are_norm )
      qtype=1; % Component normalized q's are column vectors
  
    elseif ( rows_are_norm && ~cols_are_norm)
      qtype=2; % Component normalized q's are row vectors
  
    else
      qtype=3; % Component normalized q's are either columns or rows
  
    end
    
  end

end