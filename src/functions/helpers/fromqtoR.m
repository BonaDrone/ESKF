function [R] = fromqtoR(q)
qw = q(1);
qv = q(2:4);
%R = (qw^2-qv'*qv)*eye(3) + 2*qv*qv' + 2*qw*skew(qv);
R = (qw^2-transpose(qv)*qv)*eye(3) + 2*qv*transpose(qv) + 2*qw*skew(qv);