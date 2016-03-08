function m = cpm(r)
%cpm(r) gives 3x3 anti-symmetric cross product matrix for 3x1 or 1x3 r
%
%   This function is used for development and testing, but may not be called
%   from production code due to optimization.
%
% Copyright (C) 2013 Marsette A. Vona

m = [0, -r(3), r(2); r(3), 0, -r(1); -r(2), r(1), 0];
end
