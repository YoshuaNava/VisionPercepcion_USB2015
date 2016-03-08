function j = drlog(m,fmt)
%j = drlog(m) Jacobian of rlog(m)
%
%  The optional argument fmt controls the format of the output.  It may either
%  be 't' (default) or 'm'.  In the former case the output is a 3x[3x3] tensor
%  stored as a Matlab 3D array; j(:,:,i) is the Jacobian of (rlog(m))(i) with
%  respect to m.  In the latter case the output is a 3x9 matrix where j(i,:) is
%  the Jacobian of (rlog(m))(i) with respect to m(:).
%
%  Numerically stable as m approaches identity.
%
% Copyright (C) 2013 Marsette A. Vona

if (nargin<2); fmt = 't'; end

[~,j] = rlog(m,fmt);

% This does work but it does not compare correctly to numerical derivative.
% It's also about 30% slower.
%jm = pinv(reshape(drexp(rreparam(rlog(m))),9,3));
%j = zeros(3,3,3);
%for i=1:3; j(:,:,i) = reshape(jm(i,:),3,3); end

end

