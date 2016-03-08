function [ss, hss, ssm, pp, hpp] = demospl(fn, varargin)
% demospl(fn) load, plot, and segment one or more datasets.
%
%   Argument fn may either be a string or a cell array of strings giving the
%   name(s) of files to load.
%
%   Returns
%
%   * a 1xS cell array of the MxNx3 loaded sample sets in ss
%
%   * a 1xS array of the corresponding sampleplot handles in hss
%
%   * a 1xS cell array of adjtri meshes (see samplemesh.m) in ssm
%
%   * a 1xS cell array of the 1xP fitted patches for each dataset in pp
%
%   * a 1xS cell array of the 1xP corresponding patchplot handles in hpp
%
%   S is the number of datasets.  The number of samples M*N and patches P may
%   vary across datasets.
%
%   OPTIONS
%
%   A list of name,value pairs may be given to set options.  Unrecognized names
%   cause warnings. If a name is given more than once the last-given (rightmost)
%   value takes precedence.
%
%   Option 'sp' (default 1): whether to plot datasets in different subplots; if 
%   not, they are all plotted into the same axes.
%
%   Option 'ctr' (default [0;0;0]): centers of projection.  Either vector or
%   matrix (one center per column) for separate control of each dataset.
%
%   Option 'frustum' (default []): frustum parameters.  See 'frustum' option in
%   sampleplot().  Either vector or matrix (one frustum per column) for separate
%   control of each dataset.
%
%   Option 'sc' (default 0): whether to call samplecvt() on the loaded data.
%   Either scalar or vector for separate control of each dataset.  Note: if
%   'xform' or 'recenter' options are included in the corresponding convert
%   options (see below), they are applied to the center of projection and
%   frustum as well.
%
%   Option 'sm' (default 0): whether to call samplemesh() on the loaded data.
%   Either scalar or vector for separate control of each dataset.
%
%   Option 'ds' (default 1): whether to draw samples.  Either scalar or vector
%   for separate control of each dataset.
%
%   Option 'df' (default 0): whether to draw frusta.  Either scalar or vector
%   for separate control of each dataset.  Ignored if frustum is empty.
%
%   Option 'dc' (default 0): whether to draw covariances.  Either scalar or
%   vector for separate control of each dataset.  Ignored if frustum is empty.
%
%   Option 'dp' (default 1): whether to draw patches.  Either scalar or
%   vector for separate control of each dataset.
%
%   Option 'fit' (default 0): whether to call patchfit directly on the loaded
%   data.  Either scalar or vector for separate control of each dataset.
%   Note that patches may also be fit as part of manual or auto segmentation.
%
%   Option 'ms' (default 0): manual segmentation enable.  Fits multiple patches
%   to user selected neighborhoods.  Either scalar or vector for separate
%   control of each dataset.
%
%   Option 'as' (default 0): auto segmentation enable.  Fits multiple patches to
%   automatically selected neighborhoods.  Either scalar or vector for separate
%   control of each dataset.
%
%   Option 'slopts' (default 1): options to pass to sampleload.  Scalar,
%   vector, cell array, or function handle.  Scalar or vector to use a preset
%   for all/each dataset.  Cell array to use given options for all datasets.
%   Function handle must take index of dataset and return cell array of options
%   for that dataset.
%
%   Option 'smopts' (default 1): options to pass to samplemesh.  Same format as
%   slopts.
%
%   Option 'spopts' (default 1): options to pass to sampleplot.  Same format as
%   slopts.  'decimate', 'lowres', and 'subsample' options may be specified as
%   'auto'.
%
%   Option 'scopts' (default 1): options to pass to samplecvt.  Same format as
%   slopts.  'xform' option may be specified as 'auto'.
%
%   Option 'fpopts' (default 1): options to pass to frustumplot.  Same format as
%   slopts.  '?c' color options may be specified as 'auto'.
%
%   Option 'rcopts' (default 1): options to pass to rangecovar.  Same format as
%   slopts.
%
%   Option 'cpopts' (default 1): options to pass to covarplot.  Same format as
%   slopts.  '?c' color options may be specified as 'auto'.
%
%   Option 'pfopts' (default 1): options to pass to patchfit.  Same format as
%   slopts.
%
%   Option 'ppopts' (default 1): options to pass to patchplot.  Same format as
%   slopts.  '?c' color options may be specified as 'auto'.
%
%   Option 'asopts' (default 1): options to pass to autoseg.  Same format as
%   slopts.
%
%   Option 'av' (default 1): whether to draw plot axes.
%
%   Option 'dt' (default 1): whether to draw plot title.
%
%   Option 'colors' (default {'b','g','r','c','m','y','k'}): colors to cycle for
%   'auto'.
%
%   Option 'spmax' (default 30000): max samples to plot per dataset for 'auto'.
%
%   Option 'cpmax' (default 100): max covars to plot per dataset for 'auto'.
%
% Copyright (C) 2013 Marsette A. Vona and Dimitrios Kanoulas

tstart = tic(); %timing

% configuration %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

maxnc = 4; % maximum number of subplot columns
asdbg = 0; % autoseg debug

% option presets %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% sampleload
dsl1 = {}; 
dsl = {dsl1};

% samplemesh
dsm0 = {'maxedge',0.05,'maxedgeratio',5};%'dbg',1
dsm1 = [dsm0 {'edgedetect',2}];
dsm2 = [dsm1 {'decimate',2}];
dsm3 = dsm0;
dsm = {dsm1, dsm2, dsm3};

% sampleplot
dsp0 = {'as',0.1,'ps','auto','pc','auto','fb',1};
dsp1 = [dsp0 {'decimate','auto'}];
dsp2 = [dsp0 {'decimate',3}];
dsp3 = [dsp0 {'decimate',2}];
dsp4 = [dsp3 {'kv',1}];
dsp5 = [dsp3 {'df',1}];
dsp6 = [dsp0 {'df',1}];
dsp7 = [dsp0 {'df',1,'kv',1}];
dsp8 = dsp0;
dsp9 = [dsp0 {'zoom',1.5}];
dsp10 = [dsp9 {'df',1}];
dsp = {dsp1, dsp2, dsp3, dsp4, dsp5, dsp6, dsp7, dsp8, dsp9, dsp10};

% samplecvt
dsc1 = {'xform','auto'};
dsc2 = {'recenter',1};
dsc3 = {'recenter',1,'xform',[[-pi/2,0,0]',[0,0,0]']}; % rock.ply
dsc4 = {'xform',[[-pi/1.3,pi/7,-pi/9]',[0,0,0]']};
dsc5 = {'recenter',1,'xform',[[-pi/1.6,pi/7,-pi/9]',[0,0,0]']};
dsc6 = [dsc5 ...
        {'markplane',[0,0.64,0.76,1],'marksphere',[0,0,1.8,1]}]; % newrock.pcd
dsc7 = [dsc6 {'decimate',4}];
dsc = {dsc1, dsc2, dsc3, dsc4, dsc5, dsc6, dsc7};

% frustumplot
dfp1 = {'da',0,'uc','auto'};
dfp = {dfp1};

% rangecovar
rc_kinect = [0.11 0.03 7.5e-2 580];
drc1 = {'stereo',rc_kinect};
drc = {drc1};

% covarplot
dcp1 = {'p',0.95,'de',2,'da',0,'ns',4,'decimate','auto'};
dcp = {dcp1};

% patchfit
ppss = 0.005; ppas = 0.01; ssmax = 50;
dfp0 = {'ppss',ppss,'ppas',ppas,'ssmax',ssmax};
% fit planes with ellipse boundaries, with dbg gfx
dpf1 = [dfp0 {'dbg',6,'st','p','db',0.08,'fb',1}];
% fit paraboloids with fixed circle boundaries, with dbg gfx
dpf2 = [dfp0 {'dbg',6,'st','a','db',0.04,'fb',0}];
% fit paraboloids with fixed circle boundaries, no dbg for timing
dpf3 = [dfp0 {'dbg',0,'st','a','db',0.04,'fb',0}];
% fit paraboloids with ellipse boundaries, no dbg for timing
dpf4 = [dfp0 {'dbg',0,'st','a','db',0.04,'fb',1}];
% fit paraboloids with ellipse boundaries, no dbg for timing, no center constr
dpf5 = [dfp0 {'dbg',0,'st','a','db',0.04,'fb',1,'ccon','no'}];
% fit paraboloids with ellipse boundaries, with dbg messages
dpf6 = [dfp0 {'dbg',1,'st','a','db',0.04,'fb',1,'ktol',0.3}];
% fit planes with ellipse boundaries
dpf7 = [dfp0 {'dbg',0,'st','p','db',0.08,'fb',1}];
% fit planes with ellipse boundaries, with dbg gfx and msgs
dpf8 = [dfp0 {'dbg',2,'st','p','db',0.08,'fb',1}];
dpf = {dpf1,dpf2,dpf3,dpf4,dpf5,dpf6,dpf7,dpf8};

% patchplot
dpp1 = {'ss',ppss,'gd',2,'aw',1,'as',ppas,'bw',2,'bc','r','gw',1,...
        'gc','r','df',0};
dpp = {dpp1};

% autoseg
%rpshow = 2; asdbg = 2;
%rpshow = 0; asdbg = 1;
%rpshow = 0; asdbg = 3;
rpshow = 2; asdbg = 3;
ssr = 0.05; ssk = 5000;
das0 = {'rpshow',rpshow,'ssr',0.05,'ssk',5000,'dbg',asdbg};
das1 = [das0 {'ssmethod','kdtree'}];
das2 = [das0 {'ssmethod','bfsdd'}];
das3 = [das0 {'ssmethod','bfsmd'}];
das4 = [das1 {'rpshow',2,'dbg',2}]; % pauses
das5 = [das1 {'rpshow',2,'dbg',3}]; % no pauses
das6 = [das1 {'rpshow',0,'dbg',3}]; % no pauses, delete rejected
das = {das1 das2 das3 das4 das5 das6};

% argument handling %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
sp = 1;
ctr = [0 0 0]'; frustum = [];
sc = 0; sm = 0;
ds = 1; df = 0; dc = 0; dp = 1;
fit = 0; ms = 0; as = 0;
slopts = 1; smopts = 1; spopts = 1; scopts = 1;
fpopts = 1; rcopts = 1; cpopts = 1;
pfopts = 1; ppopts = 1; asopts = 1;
av = 1; dt = 1; colors = {'b','g','r','c','m','y','k'};
spmax = 30000; cpmax = 100;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'sp'; sp = v; case 'ctr'; ctr = v; case 'frustum'; frustum = v;
      case 'sc'; sc = v; case 'sm'; sm = v;
      case 'ds'; ds = v; case 'df'; df = v;
      case 'dc'; dc = v; case 'dp'; dp = v;
      case 'fit'; fit = v; case 'ms'; ms = v; case 'as'; as = v;
      case 'slopts'; slopts = v; case 'smopts'; smopts = v;
      case 'spopts'; spopts = v; case 'scopts'; scopts = v;
      case 'fpopts'; fpopts = v; case 'rcopts'; rcopts = v;
      case 'cpopts'; cpopts = v; case 'pfopts'; pfopts = v;
      case 'ppopts'; ppopts = v; case 'asopts'; asopts = v;
      case 'av'; av = v; case 'dt'; dt = v;
      case 'colors'; colors = v;
      case 'spmax'; spmax = v; case 'cpmax'; cpmax = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

% elaborate inputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (ischar(fn)); fn = {fn}; end % allow a single filename
nd = length(fn); % number of datasets

[~,nct] = size(ctr);
[~,nfr] = size(frustum);

if (sp)
  ndr = ceil(nd/maxnc); % subplot rows
  if (ndr>1); ndc = maxnc; else ndc = nd; end % subplot cols
else ndr = 1; ndc = 1; end

% helper functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  function o = isv(v)
  o = (isvector(v)&&(length(v)>1));
  end

% auto opt handling %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nextc = 1; % next auto color
xformrr = eye(3,3); xformt = zeros(3,1); % most recent specified xform
firstad = -1; firstaps = -1; % first dataset with auto decimate or point size

  function o = getopts(i,opts,dopts,sz,maxp,msg)
  % get options (cell array o) for an operation on dataset i
  % opts specifies the options, see toplevel doc for the possible ways
  % dopts are sets of default options
  % sz is the size of the data matrix for auto decimate/lowres
  % maxp is the max number of points for auto subsample
  % msg is used to tag info messages
 
  % resolve option array
  if (isscalar(opts)); o = dopts(opts);
  elseif (isv(opts)); o = dopts(opts(i));
  elseif (iscell(opts)); o = opts;
  elseif (isa(opts,'function_handle')); o = opts(i);
  end
  o = o{:};
 
  if (isempty(maxp)); maxp = 0; end

  ac = colors{nextc}; % auto color
  ass = maxp; % auto subsample
  ax = [xformrr,xformt]; % auto xform
  
  % auto decimate
  np = prod(sz); md = (~isempty(sz)&&all(sz>1));
  if (firstad>0); ad = firstad;
  elseif (maxp>0); ad = np/maxp; if (md); ad = sqrt(ad); end; ad = ceil(ad);
  else ad = 0;
  end
  
  % auto point size
  if (firstaps>0); aps = firstaps+3;
  elseif (np>maxp); aps = 2;
  else aps = 6;
  end
 
  % replace auto args
  ii = find(strcmpi(o,'auto'));
  for i=ii
    if ((i>1)&&ischar(o{i-1}))
      n = o{i-1}; % arg name
      if ((length(n)==2)&&(n(2)=='c')) % handle an auto color
        o{i} = ac; nextc = nextc+1; if (nextc>length(colors)); nextc = 1; end
      else
        switch(n) % handle some other auto
          case 'ps'; o{i} = aps; if (firstaps<0); firstaps = aps; end
          case {'decimate','lowres'};
            o{i} = ad; if (firstad<0); firstad = ad; end
            if (ad>0); fprintf('%s auto %s by %d\n',msg,n,ad); end
          case 'subsample';
            o{i} = ass;
            if (ass>0); fprintf('%s auto %s by %d\n',msg,n,ass); end
          case 'xform'; o{i} = ax;
        end % switch
      end % not an auto color
    end % got arg name
  end % for each auto arg
  
  end % getopts

% patchfit wrapper %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nf = 0; % total num fits
fp = 0; % total num fitted points
fs = 0; % total fit seconds
pfo = {}; % patchfit options
ppo = {}; % patchplot options
dfpp = true; % whether to call patchplot, driven per dataset by df
dfverbose = true; % whether to emit messages from dofit

  function [p] = dofit(xx,yy,zz,r,opts)
  % wraps patchfit to emit extra messages, call patchplot, and collect stats
  % xx,yy,zz are the data to fit
  % r is either empty or an initial guess for the patch radius 'db'
  % opts are extra options to patchfit, pfo will be appended
 
  opts = [opts,pfo];

  nd = numel(xx);
 
  % emit messages about ssmax and bsmax
  ii = find(strcmpi(opts,'ssmax'),1,'last');
  ssmx = 0; if (~isempty(ii)); ssmx = opts{ii+1}; end
  if (dfverbose&&(ssmx>0)&&(nd>ssmx))
    fprintf('patchfit subsampling %d of %d for surface fit\n',ssmx,nd);
  end
  
  ii = find(strcmpi(opts,'bsmax'),1,'last');
  bsm = 0; if (~isempty(ii)); bsm = opts{ii+1}; end
  if (bsm>ssmx); bsm = ssmx; end
  if ((bsm>0)&&(nd>bsm))
    fprintf('patchfit subsampling %d of %d for boundary fit\n',bsm,nd);
  end

  if (~isempty(r)) % init patch radius to pick radius
    ii = find(strcmpi(opts,'db'),1,'last');
    if (~isempty(ii)); opts{ii+1} = r; else opts = {opts{:},'db',r}; end
    if (dfverbose); fprintf('patchfit initial radius %d\n',r); end
  end
  
  p = patchfit(xx,yy,zz,opts{:});
  
  if (isfield(p,'hp')&&(~isempty(p.hp))); delete(p.hp); p.hp = []; end
  if (dfpp&&~isempty(p))
    hp = patchplot(p,ppo{:}); p.hp = hp;
    drawnow(); % expensive
  else hp = []; p.hp = []; end
  
  sn = nan; res = nan; sec = nan;
  if (isfield(p,'fitsn')&&(~isempty(p.fitsn))); sn = p.fitsn; end
  if (isfield(p,'residual')&&(~isempty(p.residual))); res = p.residual; end
  if (isfield(p,'fitsec')&&(~isempty(p.fitsec))); sec = p.fitsec; end

  ii = find(strcmpi(opts,'dores'),1,'last');
  rtype = 'alg';
  if (~isempty(ii)&&ischar(opts{ii+1})); rtype = opts{ii+1}; end

  if (dfverbose)
    fprintf('patchfit %d samples, %s residual %g, time %gs\n',sn,rtype,res,sec);
  end

  nf = nf+1; fp = fp+p.fitsn; fs = fs+p.fitsec;
  
  end %dofit

% demo each dataset %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear figure window
persistent fig;
if (isempty(fig)); fig = figure(); end
if (ishandle(fig)); clf(fig); else figure(fig); end % make current w/o focus
set(fig,'Color','w');

% return arrays
ss = cell(1,nd); ssm = cell(1,nd); pp = cell(1,nd);
hss = zeros(1,nd); hpp = cell(1,nd);

for i=1:nd % for each dataset

  % load the dataset
  fprintf('loading %s...',fn{i});
  slo = getopts(i,slopts,dsl,[],[],'sampleload');
  [x, y, z] = sampleload(fn{i},slo{:});
  sz = size(x); np = sz(1)*sz(2);
  vi = (((imag(x)==0)&(imag(y)==0)&(imag(z)==0))&...
        ~(isnan(x)|isnan(y)|isnan(z))&...
        ~(isinf(x)|isinf(y)|isinf(z)));
  nbp = np-sum(vi(:));
  fprintf('done\n%dx%d, %d points, %d bad\n',sz(1),sz(2),np,nbp);

  % determine center and frustum
  if (nct>1); ct = ctr(:,i); else ct = ctr(:,1); end
  fr = []; if (nfr>1); fr = frustum(:,i); elseif (nfr>0); fr = frustum(:,1); end

  % figure out what we need to do
  dosc = ((isv(sc)&&sc(i))||(~isv(sc)&&sc)); % sample convert
  dodf = ((isv(df)&&df(i))||(~isv(df)&&df)&&~isempty(fr)); % draw frustum
  dodc = ((isv(dc)&&dc(i))||(~isv(dc)&&dc)); % draw covariance
  doft = ((isv(fit)&&fit(i))||(~isv(fit)&&fit)); % patchfit
  dfpp = ((isv(dp)&&dp(i))||(~isv(dp)&&dp)); % draw patches
  doms = ((isv(ms)&&ms(i))||(~isv(ms)&&ms)); % manual segmentation
  doas = ((isv(as)&&as(i))||(~isv(as)&&as)); % auto segmentation
  
  dosm = all(sz>1)&&((isv(sm)&&sm(i))||(~isv(sm)&&sm)||doas||doms); % samplemesh
  dods = ((isv(ds)&&ds(i))||(~isv(ds)&&ds)||doms); % draw samples
  dorc = (dodc||doft||doms||doas); % range covar
 
  % filter the dataset with samplecvt
  if (dosc)
    
    sco = getopts(i,scopts,dsc,sz,np,'samplecvt');
    
    % must apply any given xform to ct and frustum
    ii = find(strcmpi(sco,'xform'),1,'last'); rr = eye(3,3);
    if (~isempty(ii))
      xform = sco{ii+1};
      [xr, xc] = size(xform);
      if ((xc==4)&&((xr==3)||(xr==4))); t = xform(1:3,4); rr = xform(1:3,1:3);
      elseif (xc==2); t = xform(:,2); rr = rexp(xform(:,1));
      else error('xform must be 3x4, 4x4, or 3x2');
      end
      ct = rr*ct+t;
      if (~isempty(fr)); fr(3:5) = rr*fr(3:5); fr(6:8) = rr*fr(6:8); end
      xformrr = rr; xformt = t; % for auto xform
    end
   
    % must apply any given recenter to ct and xformt
    ii = find(strcmpi(sco,'recenter'),1,'last');
    if (~isempty(ii)&&sco{ii+1})
      ofs = [nanmean(x(:));nanmean(y(:));nanmean(z(:))];
      ct = ct-ofs;
      xformt = xformt-rr*ofs;
    end

    [x, y, z] = samplecvt('x',x,'y',y,'z',z,sco{:});
    
    npwas = np; sz = size(x); np = sz(1)*sz(2);

    nbpwas = nbp;
    vi = (((imag(x)==0)&(imag(y)==0)&(imag(z)==0))&...
          ~(isnan(x)|isnan(y)|isnan(z))&...
          ~(isinf(x)|isinf(y)|isinf(z)));
    nbp = np-sum(vi(:));

    fprintf('samplecvt discarded %d points',npwas-np);
    if (npwas==np); fprintf(', marked %d bad',nbp-nbpwas);
    else fprintf('\n'); end
    
  end % samplecvt

  ss{i} = cat(3,x,y,z); % save dataset to output

  % sample mesh
  sm = [];
  if (dosm)
    smo = getopts(i,smopts,dsm,sz,spmax,'samplemesh');
    sm = samplemesh('x',x,'y',y,'z',z,smo{:});
    ssm{i} = sm;
  else ssm{i} = []; end

  % activate subplot for drawing
  if (sp); subplot(ndr,ndc,i); end
  if (dt); title(fn{i},'interpreter','none'); end
  if (~av); a = gca(); set(a,'Visible','off'); end 
  
  % plot samples (and possibly mesh)
  if (dods)
    spo = getopts(i,spopts,dsp,sz,spmax,'sampleplot');
    hss(i) = sampleplot('x',x,'y',y,'z',z,'adjtri',sm,spo{:});
    hold('on'); drawnow();
    hs = findobj(get(hss(i),'Children'),'flat','Tag','samples');
  end
  
  % draw frustum
  if (dodf)
    fpo = getopts(i,fpopts,dfp,sz,np,'frustumplot');
    frustumplot('fov',fr(1:2),'p',fr(3:5),'u',fr(6:8),'c',ct,fpo{:});
    drawnow();
  end
 
  % rangecovar
  if (dorc)  
    [mx,my,mz,r] = samplecvt('x',x,'y',y,'z',z);
    rco = getopts(i,rcopts,drc,sz,np,'rangecovar');
    [cxx,cyy,czz,cxy,cyz,cxz] = rangecovar(mx,my,mz,r,rco{:});
  end

  % covarplot
  if (dodc)
    cpo = getopts(i,cpopts,dcp,sz,cpmax,'covarplot');
    covarplot('cxx',cxx,'cyy',cyy,'czz',czz,'cxy',cxy,'cyz',cyz,'czz',czz,...
              'cx',ct(1),'cy',ct(2),'cz',ct(3),cpo{:});
    drawnow();
  end

  % get patch fit and plot opts if we'll need them
  if (doms||doas||doft)
    pfo = getopts(i,pfopts,dpf,sz,np,'patchfit');
    ppo = getopts(i,ppopts,dpp,sz,np,'patchplot');
  end

  pp{i} = []; % collecting patches

  % manual segmentation
  if (doms)
    fprintf('manual segmentation\n');
    dfverbose = true;
    msp = manseg(x,y,z,...
                 'cxx',cxx,'cyy',cyy,'czz',czz,'cxy',cxy,'cyz',cyz,'cxz',cxz,...
                 'nosplot',1,'nopplot',1,'dsel',hs,'ff',@dofit,'adjtri',sm);
    pp{i} = [pp{i}, msp]; hpp{i} = [hpp{i}, arrayfun(@(p)(p.hp),msp)];
  end

  % auto segmentation
  if (doas)
    fprintf('auto segmentation\n');
    aso = getopts(i,asopts,das,sz,np,'autoseg');
    dfverbose = false;
    asp = autoseg(x,y,z,...
                  'cxx',cxx,'cyy',cyy,'czz',czz,...
                  'cxy',cxy,'cyz',cyz,'cxz',cxz,...
                  'nosplot',1,'nopplot',1,'ff',@dofit,'adjtri',sm,aso{:});
    pp{i} = [pp{i}, asp{:}];
    asphp = cellfun(@(p)(p.hp),asp,'UniformOutput',false);
    hpp{i} = [hpp{i}, asphp{:}];
  end

  if(~ishandle(a)); break; end % e.g. user closed window
  
  if (doft) % fit whole dataset
    dfverbose = true;
    p = dofit(x,y,z,[],... % no initial radius 
              {'cxx',cxx,'cyy',cyy,'czz',czz,'cxy',cxy,'cyz',cyz,'cxz',cxz});
    pp{i} = [pp{i}, p]; hpp{i} = [hpp{i}, p.hp];
  end
  
  pp{i} = arrayfun(@(p)(patchchk(p,'elaborate','all')),pp{i});

end % for each dataset

fprintf('%d patchfits, total %d pts, in %g sec\n', nf, fp, fs);
if (nf>0)
  fprintf('avg %g pts/fit, %g sec/fit, %g sec/pt\n', fp/nf, fs/nf, fs/fp);
end

fprintf('demospl: %gs\n',toc(tstart));

end
