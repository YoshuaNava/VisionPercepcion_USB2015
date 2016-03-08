function testapprox()
%testapprox tests small-angle approximations alpha, beta, dalpha, dbeta
%
%   The alpha and beta expressions of theta in the Rodrigues function and
%   its derivative may become numerically unstable for small theta. In that
%   regime, we substitute an alternate approximation which is more
%   numerically stable.
%
%   Four plots are shown, one for each expression.  The limit of the
%   expression is shown as a horizontal blue line, and the threshold below
%   which the approximation is used is shown as a vertical magenta line.
%
%   The original function is evaluated at a set of samples represented with
%   red x points.  The approximation function is also evaluated at several
%   samples and plotted with cyan + points.  The composite function, which
%   switches to the approximation below the threshold, is then plotted with
%   a green line.
%
%   The implementation uses the correspondingly named functions for each of
%   the four expressions, however, these are not always used in the
%   implementation due to optimizations.
%
% Copyright (C) 2013 Marsette A. Vona

n = 20; pn = 1;

persistent fig;
if (isempty(fig)); fig = figure(); end
figure(fig); clf();% make fig current
set(fig,'Color','w');

  function p(fn)
  subplot(2,2,pn); pn = pn+1;
  f = str2func(fn);
  thresh = f(0,'thresh'); lim = f(0,'lim');
  x = linspace(thresh/10,thresh*2,n);
  plot(x,f(x),'g-',x,f(x,'f'),'rx',x,f(x,'fl'),'c+'); hold on;
  x = linspace(thresh/10,thresh,n);
  plot(x,f(x,'f'),'rx');
  drawnow; axis manual;
  plot([thresh, thresh],get(gca(),'YLim'),'m-');
  plot(get(gca(),'XLim'),[lim,lim],'b-');
  title(fn)
  end

fn = {'alpha', 'beta', 'dalpha', 'dbeta'};
for fi = 1:length(fn); p(fn{fi}); end

end
