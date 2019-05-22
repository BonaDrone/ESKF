function [p, v, q, ab, wb, qw, qv] = assignState(x)

p  = x(1:3);
v  = x(4:6);
q  = x(7:10);
ab = x(11:13);
wb = x(14:16);

qw = q(1);
qv = q(2:4);

end
