function [om] = Omega(w)
om = [0, -w'; w -skew(w)];
end
