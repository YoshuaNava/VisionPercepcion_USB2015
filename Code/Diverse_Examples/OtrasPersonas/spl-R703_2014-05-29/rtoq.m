function q = rtoq(r)
% r = rtoq(r) converts a rotation vector to a quaternion.
%
%   Input r is a 1x3 or 3x1 rotation vector.
%
%   Output is a 1x4 or 4x1 unit quaternion corresponding to r in order w, x, y,
%   z.
%
% Copyright (C) 2013 Marsette A. Vona

t = norm(r); t2 = t/2; c2 = cos(t2); s2 = sin(t2); 
if (t > eps); r = r/t; end
q = [c2; s2*r(:)];
[r, c] = size(r); if (c > r); q = q'; end

end
