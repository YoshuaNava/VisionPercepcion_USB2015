function r = qtor(q)
% r = qtor(q) converts a quaternion to a rotation vector
%
%   Input q is a 1x4 or 4x1 quaternion in order w, x, y, z.  It will be
%   internally normalized but degenerate quaternions with norm(q) near zero may
%   give unpredictable results.
%
%   Output is a 1x3 or 3x1 canonical (i.e. magnitude at most pi) rotation vector
%   corresponding to q.
%
% Copyright (C) 2013 Marsette A. Vona

q = q/norm(q); c2 = q(1); r = q(2:4); s2 = norm(r);
if (s2>eps); r = r/s2; end
r = rreparam(r*2*atan2(s2,c2));

end
