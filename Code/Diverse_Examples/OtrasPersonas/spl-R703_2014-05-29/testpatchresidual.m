function residuals = testpatchresidual(tpi,varargin)
% testpatchresidual(tpi) tests patchresidual() on each type in tpi
%
%   Return is an array of the residuals.
%
%   Extra args are passed on to patchresidual().
%
%   See testpatches() for the types of patches corresponding to tpi.
%
%   testpatchresidual(tpi) uses default patchresidual() options.
%
%   testpatchresidual() uses tpi=1:10
%
% Copyright (C) 2013 Dimitrios Kanoulas and Marsette A. Vona

% distance to perturb 
%perturb = 0.1;
perturb = 0.5;
%perturb = 5;
%perturb = 50;

% whether to randomize perturbations
%rnd = 1;
rnd = 0;

% requested number of samples
% actual number is lower - only sample rays that intersect the patch are kept
%nd = 10*10;
nd = 30*30;
%nd = 100*100;

dp = 1; % draw original patches
ds = 1; % draw samples

% patchplot options
ppo = {'aw',3,'as',1.5,'bw',2,'gw',1,'df',0};

% samplefrustum and patchsample options
cx = -1; cy = -1; cz = 6; fc = [cx cy cz]; % center of projection
fh = pi/3; fv = pi/3; fp = -fc; fp = fp./norm(fp); fu = [0 1 0]; % frustum
nh = round(sqrt(nd)); nv = nh; % sample array dimensions

% end of configuration %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (nargin < 1); tpi=1:10; end

persistent fig;
if (isempty(fig)); fig = figure(); end
if (ishandle(fig)); clf(fig); else figure(fig); end % make current w/o focus
set(fig,'Color','w');

residuals = zeros(length(tpi),1); % output
nt = 0; np = 0; ns = 0; % num tests, points, seconds

  function ok = test(p)

  % assume local = world
  assert(~any(p.r));
  assert(~any(p.c));

  % have to regen rays for every test b/c failed intersections get cleaned
  [mx, my, mz] = samplefrustum(fh, fv, nh, nv, fp, fu);
  
  % generate samples
  [r,mx,my,mz] = patchsample(p,mx,my,mz,'cx',cx,'cy',cy,'cz',cz,...
                             'clean',1,'fp',fp,'fu',fu,'fc',fc);
  x = cx+mx.*r; y = cy+my.*r; z = cz+mz.*r;
  
  % perturb sample points in the direction of the surface normal
  xyz = [x(:) y(:) z(:)]; [npp, ~] = size(xyz);
  ptb = perturb*ones(npp,1);
  if (rnd); ptb = ptb.*(2*rand(npp,1)-1); end
  xyz = xyz+repmat(ptb,1,3).*p.sn(xyz);
  x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);

  if (ds); sampleplot('x',x,'y',y,'z',z); hold('on'); end

  [res sec] = patchresidual(p,'x',x,'y',y,'z',z,'src','xyzl',varargin{:});

  %sec = tic();
  %res = patchresidualOLD(p,x,y,z,varargin{:});
  %sec = toc(sec);
  
  type = 'exact'; xres = []; ii = find(strcmpi(varargin,'type'),1,'last');
  if (~isempty(ii))
    type = varargin{ii+1};
    if (~strcmpi(type,'exact'))
      xres = patchresidual(p,'x',x,'y',y,'z',z,'src','xyzl',...
                           varargin{:},'type','exact');
    end
  end

  ok = 1; % TBD check residual
  
  fprintf('patch %d %s residual: %g, time: %gs, %g points\n',...
          nt+1,type,res,sec,npp);
  if (~isempty(xres))
    fprintf('patch %d exact residual: %g, diff: %g\n',nt+1,xres,res-xres);
  end
  
  nt = nt+1; np = np+npp; ns = ns+sec; residuals(nt) = res;

  end % test

testpatches(@test,tpi,dp,ppo);

fprintf('tested %d patches, total %d pts, in %g sec\n', nt, np, ns);
fprintf('avg %g pts/patch, %g sec/patch, %g sec/pt\n', np/nt, ns/nt, ns/np);

end
