function patches = testpatchfit(tpi,varargin)
% testpatchfit(tpi) tests patchfit() on each patch type in tpi
%
%   Return is a cell array of the fitted patches.
%
%   Extra args are passed on to patchfit().
%
%   See testpatches() for the types of patches corresponding to tpi.
%
%   Plane, sphere, and circ cyl patches are fit with the patchfit option 'st'
%   set to the corresponding type; all others are fit with 'st' set to 'a'
%   (paraboloid fit).  This can be overridden by passing 'st','foo' as an extra
%   arg; e.g. passing 'st','a' will fit all patches as paraboloids.
%
%   testpatchfit(tpi) uses default patchfit() options.
%
%   testpatchfit() uses tpi=1:10
%
% Copyright (C) 2013 Marsette A. Vona and Dimitrios Kanoulas

sidewall = 0; % create a sidewall effect on purpose

% TBD: When double-screen activated, dp=0 throws exception.
dp = 1; % draw original patches
dst = 0; % draw subtitles
df = 1; % draw frustum
dr = 1; % draw rays

% plot options for original patches
ppo = {'aw',3,'as',1.5,'bw',2,'gw',1,'df',0};

% plot options for fitted patch
fitppo = {'ss',0.2,'gd',2,'aw',1,'as',1.5,...
          'bw',2,'bc','r','gw',1,'gc','r','df',0,'fb',1};

% patchfit options
%pfo = {'docovar',1};
pfo = {};

perturb = 1; % whether to add noise
errormodel = 'stereo'; errorparams = [0.12 0.03 7.5e-2 580];

% samplefrustum and patchsample options
cx = -1; cy = -1; cz = 6; fc = [cx cy cz]; % center of projection
fh = pi/3; fv = pi/3; fp = -fc; % frustum
if(sidewall); fp = [6 1 -6]; end
fp = fp./norm(fp); fu = [0 1 0];
nh = 30; nv = 30; % sample array dimensions

% for sidewall change some settings
if (sidewall)
  dp=0; df=0; dr=0; perturb=0;
  fitppo = [fitppo {'aw',3,'gw',2}];
  pfo = [pfo {'ppas',0.5,'ccon',0}];
end

% end of configuration %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (nargin < 1); tpi = 1:10; end

if (length(tpi)>4); df = 0; end

persistent fig;
if (isempty(fig)); fig = figure(); end
if (ishandle(fig)); clf(fig); else figure(fig); end % make current w/o focus
set(fig,'Color','w');

dbg = 0; di = find(strcmpi(varargin,'dbg'),1,'last');
if (~isempty(di)); dbg = varargin{di+1}; end

isdbgmsg = ((dbg>0)&&(dbg~=5)&&(dbg~=6)&&(dbg~=7));
if (isdbgmsg); dbgmsg = @(varargin)(fprintf(varargin{:}));
else dbgmsg = @(varargin)(false); end

  function o = dbgpause(varargin)
    if (~isempty(varargin)); dbgmsg(varargin{:}); end
    o = 1;
    if ((dbg==2)||(dbg==3))
      es = input('testpatchfit eval (q <enter> quits, <enter> continues):','s');
      if (strcmpi(es,'q')); o = 0; else evalin('caller',es); end
    end
  end

  function sst(s)
  if (dst); subtitle(s); end
  end

patches = cell(length(tpi),1); % output
nt = 0; np = 0; ns = 0; % num tests, points, seconds

  function ok = test(p)
    
  sst('(original patch)');
  if (~dbgpause('original patch\n')); return; end
  
  % have to regen for every test b/c failed intersections get cleaned
  [mx, my, mz] = samplefrustum(fh, fv, nh, nv, fp, fu);
  
  % generate samples
  [r,mx,my,mz,cxx,cyy,czz,cxy,cyz,cxz] = ...
      patchsample(p,mx,my,mz,'cx',cx,'cy',cy,'cz',cz,'clean',1,...
                  'errormodel',errormodel,'errorparams',errorparams,...
                  'fp',fp,'fu',fu,'fc',fc,'perturb',perturb);
  x = cx+mx.*r; y = cy+my.*r; z = cz+mz.*r;
  
  if (df)
    frustumplot('fov',[fh, fv],'p',fp,'u',fu,'c',fc);
    hold('on');
    sst('(range sensor frustum)');
    if (~dbgpause('frustum\n')); return; end
  end
  
  if (dr) % draw rays
    h = sampleplot('x',x,'y',y,'z',z,'cx',cx,'cy',cy,'cz',cz,...
                   'dp',0,'dr',1,'fb',1);
    hold('on');
    sst('(measurement rays)');
    if (~dbgpause('measurement rays\n')); return; end
    delete(h);
    fitbbox(gca());
  end
  
  sst([]); % clear subtitle

  if (any(eq(['p','s','c'],p.s))); st = p.s; else st = 'a'; end
  pp = patchfit(x,y,z,...
                'cxx',cxx,'cyy',cyy,'czz',czz,'cxy',cxy,'cyz',cyz,'cxz',cxz,...
                'st',st,pfo{:},varargin{:});
 
  if (isempty(pp)||~isfield(pp,'fitsec')||isempty(pp.fitsec))
    ok = 0; return;
  end

  % plot fitted patch unless it was done by patchfit in a debug mode
  if (~isfield(pp,'hp')||isempty(pp.hp)); patchplot(pp,fitppo{:}); end

  res = nan;
  if (isfield(pp,'residual')&&(~isempty(pp.residual))); res = pp.residual; end

  ok = 1; % TBD check residual

  fprintf('patch %d residual: %g, time: %gs\n',nt+1,res,pp.fitsec);
  
  nt = nt+1; np = np+pp.fitsn; ns = ns+pp.fitsec; patches{nt} = pp;
  
  end % test

testpatches(@test,tpi,dp,ppo);

fprintf('tested %d patches, total %d pts, in %g sec\n', nt, np, ns);
fprintf('avg %g pts/patch, %g sec/patch, %g sec/pt\n', np/nt, ns/nt, ns/np);

end

