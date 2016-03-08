function y = alpha(t, varargin)
%alpha(theta) gives alpha of theta in the Rodrigues function and its derivative
%
%   The calculation uses a small angle approximation when needed.
%
%   Optional arguments may also be specified as for approxbase().
%
%   This function is used for development and testing, but may not be called
%   from production code due to optimization.
%
% Copyright (C) 2013 Marsette A. Vona

lim=1; 
y=approxbase(t,lim,[],@(t)(sin(t)./t),@(t)(lim-t.^2/6),varargin{:});

%~50% speedup by avoiding anon fns
%additional ~30% speedup by not using nested functions

end
