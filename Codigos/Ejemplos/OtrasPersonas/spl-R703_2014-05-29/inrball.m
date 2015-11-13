function ii = inrball(r,p,pp,invert)
% ii = inrball(r,p,pp) finds indices of points in pp in or on r-ball of point p
%
%   Argument r must be a scalar, p must be Dx1, and pp must be DxN.
%
%   Option argument invert specifies whether to return points outside the
%   ball, default 0.
%
% Copyright (C) 2013 Marsette A. Vona

if (nargin < 4); invert = 0; end

[d,np] = size(pp);

pp = pp-repmat(p,1,np);
dd = sum(pp.*pp);
if (~invert); ii = find(dd<=r*r); else ii = find(dd>r*r); end

end
