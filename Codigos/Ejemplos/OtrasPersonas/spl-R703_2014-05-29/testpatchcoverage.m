function pcts = testpatchcoverage(tpi,varargin)
% testpatchcoverage(tpi) tests patchcoverage() on each type in tpi
%
%   Return is an array of the coverage percents.
%
%   Extra args are passed on to patchcoverage().
%
%   See testpatches() for the types of patches corresponding to tpi.
%
%   testpatchcoverage(tpi) uses default patchcoverage() options
%
%   testpatchcoverage() uses tpi=1:10
%
% Copyright (C) 2013 Dimitrios Kanoulas and Marsette A Vona

dp = 1; % draw original patches
ds = 1; %draw samples
dct = 'g'; % show good cells
%dct = 'i'; % show num inside pts
%dct = 'o'; % show num outside pts
%dct = 'a'; % show cell intersection area
dcz = 0.5; % coverage grid z offset
%npp = 200; % num sample points per patch
npp = 100; % num sample points per patch
exp = 1.5; % patch bbox expansion factor for sampling
nc = 50; % requested number of cells
kop = 0.8; % probability of killing outside samples for good patches
bpp = 0.2; % probability of a bad patch
ps = 6; % point size

% patchplot options
ppo = {'bw',2,'gw',1,'df',0,'da',0,'dg',0};

% end of configuration %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (nargin < 1); tpi = 1:10; end

% setup figure window
persistent fig;
if (isempty(fig)); fig = figure(); end
if (ishandle(fig)); clf(fig); else figure(fig); end % make current w/o focus
set(fig,'Color','w');

pcts = zeros(length(tpi),1); % output
nt = 0; np = 0; ns = 0; % num tests, points, seconds
ngp = 0; cgp = 0; mincgp = inf; % good patch statistics
nbp = 0; cbp = 0; maxcbp = -inf; % bad patch statistics

  function ok = test(p)
  
  % assume local = world
  assert(~any(p.r));
  assert(~any(p.c));

  xmin = exp*p.bb(1,1); xmax = exp*p.bb(1,2);
  ymin = exp*p.bb(2,1); ymax = exp*p.bb(2,2);

  bp = rand()<bpp; % bad patch?

  x = []; y = []; z = [];
  while (length(x) < npp)

    % generate a batch of random samples
    xx = xmin+rand(npp,1)*(xmax-xmin);
    yy = ymin+rand(npp,1)*(ymax-ymin);

    % kill outside samples with probability kop for good patches
    if (~bp)
      ki = (p.bl(xx,yy)>0)&(rand(npp,1)<kop);
      x = [x;xx(~ki)]; y = [y;yy(~ki)];
    else x = xx; y = yy; end
  end

  x = x(1:npp); y = y(1:npp); z = dcz*ones(npp,1);

  if (ds); sampleplot('x',x,'y',y,'z',z,'ps',ps); hold('on'); end
  
  [pct, sec, cp, cm] = patchcoverage(p,'x',x,'y',y,'nc',nc,varargin{:});

  %sec = tic();
  %p.bdata = [x y z];
  %[pc, pcb, wc] = patchcoverageOLD(p,'cn',nc);
  %pct = 1-pc.bcp; 
  %[mr, mc] = size(pc.bcmat); lc = mc/2; rc = mc/2; tc = mr/2; bc = mr/2;
  %cp = [wc 0 0 0 0 0 lc rc tc bc];
  %cm = {1-pc.bcmat};
  %dct = 'g';
  %sec = toc(sec);

  % plot coverage grid
  p.cvgp = cp; p.cvgm = cm;
  patchplot(p,'da',0,'db',0,'dg',0,'df',0,'dc',{dct,dcz},'fb',1);
  %  setcam([0 0 40], [0 0 0], [0 1 0], 10.33);
  setcam([0.1 0.1 40], [0 0 0], [0 1 0], 10.33); % avoid aliasing issues

  ok = 1; % TBD check coverage

  if (bp); gb = 'bad'; nbp = nbp+1; cbp = cbp+pct; maxcbp = max(maxcbp,pct);
  else gb = 'good'; ngp = ngp+1; cgp = cgp+pct; mincgp = min(mincgp,pct); end

  fprintf('patch %d (%s) coverage: %g, time: %gs\n',nt+1,gb,pct,sec);
  
  nt = nt+1; np = np+npp; ns = ns+sec; pcts(nt) = pct;

  end % testresidual

testpatches(@test,tpi,dp,ppo);

fprintf('%d good patches, covergage avg=%g, min=%g\n',ngp,cgp/ngp,mincgp);
fprintf('%d bad patches, covergage avg=%g, max=%g\n',nbp,cbp/nbp,maxcbp);

fprintf('tested %d patches, total %d pts, in %g sec\n', nt, np, ns);
fprintf('avg %g pts/patch, %g sec/patch, %g sec/pt\n', np/nt, ns/nt, ns/np);

end
