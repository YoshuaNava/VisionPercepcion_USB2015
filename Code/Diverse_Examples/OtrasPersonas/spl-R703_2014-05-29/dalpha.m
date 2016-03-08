function y = dalpha(t, varargin)
%dalpha(theta) gives derivative of alpha of theta in the Rodrigues derivative
%
%   The calculation uses a small angle approximation when needed.
%
%   Optional arguments may also be specified as for approxbase().
%
%   This function is used for development and testing, but may not be called
%   from production code due to optimization.
%
% Copyright (C) 2013 Marsette A. Vona

lim=-1/3;
y=approxbase(t,lim,[],@(t)((t.*cos(t)-sin(t))./t.^3),@(t)(t.^2/30+lim), ...
  varargin{:});

%~50% speedup by avoiding anon fns
%additional ~30% speedup by not using nested functions

end

