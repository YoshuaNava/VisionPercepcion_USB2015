function s = patchprint(p,nk,nd)
% s = patchprint(p) prints patch primary parameters as a string
%
%   s = patchprint(p,nk) forces nk
%   s = patchprint(p,nk,nd) forces nk and nd
%
%   Prints to the command window when no return values requested.
%
% Copyright (C) 2013 Marsette A. Vona

prec = 4;

if (nargin < 3) nd = length(p.d); end
if (nargin < 2) nk = length(p.k); end

s = sprintf('s=%s b=%s k=%s d=%s\nr=%s c=%s', p.s, p.b,...
            mat2str(p.k(1:nk),prec), mat2str(p.d(1:nd),prec), ...
            mat2str(p.r,prec), mat2str(p.c,prec));

if (nargout < 1) fprintf('%s\n',s); end

end
