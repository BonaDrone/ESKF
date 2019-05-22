function [Q] = Qmat(q)

qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

Q = 0.5*[-qx -qy -qz;...
           qw -qz  qy;...
           qz  qw -qx;...
          -qy  qx  qw];
      
end