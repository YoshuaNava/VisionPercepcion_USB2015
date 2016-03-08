function y = approxbase(t,lim,thresh,f,fl,opt)
%approxbase(t,lim,thresh,f,fl) common impl for [d]alpha(t), [d]beta(t)
%
%   t is either a scalar or vector of abscissas at which to evaluate function
%   f().  Function fl() is used instead for any abscissas less than thresh,
%   which defaults to eps(lim)^(1/4)*10 if passed as [].  Further, the 0
%   abscissa always evaluates to lim.
%
%   approxbase(t,lim,thresh,f,fl,'lim') just returns lim.
%
%   approxbase(t,lim,thresh,f,fl,'thresh') just returns thresh.
%
%   approxbase(t,lim,thresh,f,fl,'f') always uses f().
%
%   approxbase(t,lim,thresh,f,fl,'fl') always uses fl().
%
%   This function is used for development and testing, but may not be called
%   from production code due to optimization.
%
% Copyright (C) 2013 Marsette A. Vona

if (isempty(thresh)); thresh = eps(lim)^(1/4)*10; end

if (nargin < 6)
  if (isscalar(t)) %~25% speedup for scalar case
    y = lim; if (t >= thresh); y = f(t); elseif (t > 0) y = fl(t); end
  else
    y = zeros(size(t));
    ti = (t >= thresh); y(ti) = f(t(ti));
    ti = ((0 < t) & (t < thresh)); y(ti) = fl(t(ti));
    ti = (t == 0); y(ti) = lim;
  end
else
  switch (opt)
    case 'lim'; y = lim;
    case 'thresh'; y = thresh;
    case 'f'; y = f(t);
    case 'fl'; y = fl(t);
    otherwise; error('unexpected option %s',opt);
  end
end

end
