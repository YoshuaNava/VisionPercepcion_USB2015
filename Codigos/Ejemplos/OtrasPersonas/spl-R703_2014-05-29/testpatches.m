function testpatches(tfun, tpi, dp, ppo)
%testpatches(tfun, tpi, dp, ppo) runs tests on patches of different types
%
%   tfun must be the handle of a function expecting one argument, the patch
%   to test, and returning a boolean indicating test success.
%
%   tpi is a vector of indices to into an internally generated array of
%   test patches. Indices 1:10 correspond to a test patch of each type ee,
%   he, yr, oc, pc, pe, pr, pq, sc, cr in order.
%
%   dp indicates whether to draw the test patches. If dp=1, each test patch
%   will be plotted in its own axes before the corresponding call to tfun.
%   The axes will remain current with hold on so that tfun may add
%   graphics, if desired. dp=2 sets up axes but does not actually draw the
%   patches.  dp=0 disables graphics.
%
%   ppo is a cell array of name, value pairs to pass to patchplot().
%
%   testpatches(tfun, tpi, dp) uses default ppo
%
%   testpatches(tfun, tpi) defaults dp=1
%
%   testpatches(tfun) defaults tpi=1:10
%
%   testpatches() defaults tfun=@(p)(1)
%
% Copyright (C) 2013 Marsette A. Vona

% adjust these to control which plots are drawn
first = 1; last = 10;

maxnc = 4; % maximum number of subplot columns
dss = 0.4; % default sample size

defppo = {'aw',3,'as',1.5,'bw',2,'gw',1};

av = 0; % draw axes
dt = 0; % draw title

% end of configuration %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (nargin < 1); tfun = @(p)(1); end
if (nargin < 2); tpi = 1:10; end
if (nargin < 3); dp = 1; end
if (nargin < 4); ppo = defppo; end

% 10 primary test patches
i=1;
p(i).name = 'elliptic paraboloid';
p(i).s = 'e'; p(i).b = 'e'; p(i).k = [-1 -2]; p(i).d = [1.5 1];
p(i).ss = 0.5*dss; p(i).gd = 2;

i=i+1;
p(i).name = 'hyperbolic paraboloid';
p(i).s = 'h'; p(i).b = 'e'; p(i).k = [0.5 -1]; p(i).d = [1.5 1];
p(i).ss = 0.5*dss; p(i).gd = 2;

i=i+1;
p(i).name = 'cylindric paraboloid';
p(i).s = 'y'; p(i).b = 'r'; p(i).k = -2; p(i).d = [2 0.8];
p(i).ss = 0.5*dss; p(i).gd = 4;

i=i+1;
p(i).name = 'circular paraboloid';
p(i).s = 'o'; p(i).b = 'c'; p(i).k = -1; p(i).d = 1.8; p(i).ss = 0.5*dss;
p(i).gd = 4;

i=i+1;
p(i).name = 'plane (circle)';
p(i).s = 'p'; p(i).b = 'c'; p(i).d = 2; p(i).ss = dss;
p(i).gd = 2;

i=i+1;
p(i).name = 'plane (ellipse)';
p(i).s = 'p'; p(i).b = 'e'; p(i).d = [1.7 2.5]; p(i).ss = dss;
p(i).gd = 2;

i=i+1;
p(i).name = 'plane (aa rect)';
p(i).s = 'p'; p(i).b = 'r'; p(i).d = [2 2.5]; p(i).ss = 2*dss;

i=i+1;
p(i).name = 'plane (convex quad)';
p(i).s = 'p'; p(i).b = 'q'; p(i).d = [2.3 2 1.7 2.3 pi/4]; p(i).ss = 2*dss;

i=i+1;
p(i).name = 'sphere';
p(i).s = 's'; p(i).b = 'c'; p(i).k = -1/2; p(i).d = 1.8; p(i).ss = dss;
p(i).gd = 2;

i=i+1;
p(i).name = 'circular cylinder';
p(i).s = 'c'; p(i).b = 'r'; p(i).k = -1/2; p(i).d = [2 1.8];
p(i).ss = 0.5*dss; p(i).gd = 4;

% 5 negative curvature variants
pn = p([1,3,4,9,10]);
for i=1:length(pn); pn(i).k = -pn(i).k; end
p = [p, pn];

% TBD 6*4 any boundary variants

n = length(tpi); % number of test patches
nr = ceil(n/maxnc); % num axes rows
if (nr>1); nc = maxnc; else nc = n; end % num axes cols

% run tests
t = 1;
for i = tpi;
  tp(i) = patchchk(p(i),'elaborate','all');
  if (dp>0)
    subplot(nr,nc,t);
    if (dp==1); patchplot(tp(i), ppo{:}); hold('on'); end
    if (dt); title(tp(i).name); end
    if (~av); set(gca(),'Visible','off'); end
  end
  if (~tfun(tp(i))); error('test %d failed (test patch %d)', t, i); end
  t = t+1;
end

end
