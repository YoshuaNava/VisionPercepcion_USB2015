function [kp, rp] = autoseg(x,y,z,varargin)
% kp = autoseg(x,y,z) automatically segments a dataset into a set of fit patches
%
%   Arguments x, y, z must be real matrices of the same size MxN giving the
%   point sample coordinates in world frame. Variances and covariances may
%   optionally be specified for each point, see OPTIONS below.
%
%   The data is plotted with sampleplot() into the curent axes, or a new figure
%   window is opened if none is current, unless the option 'noplot' is given.
%   Neighborhood selection is handled by samplesearcher(), see the 'ssmethod'
%   option.  Patches are fit with patchfit(), see the 'ff' option.
%
%   The return kp is Px1 where P are the number of fitted patches kept by the
%   algorithm.  Optional return rp is Rx1 where R are the number of patches
%   rejected by the algorithm.  Each returned patch has the optional field hp
%   set to the corresponding patchplot() graphics handle if any.
%
%   Also see manseg.m.
%
%   OPTIONS
%
%   Options 'cxx', 'cyy', 'czz', 'cxy', 'cyz', 'cxz' (default 0): covariance of
%   the data. May each either be scalar or MxN.
%
%   Option 'nosplot' (default 0): whether to skip calling sampleplot(), e.g. if
%   the data has already been plotted.  This enables using non-standard options
%   for the plot.
%
%   Option 'nopplot' (default 0): whether to skip calling patchplot()
%
%   Option 'rpshow' (default 0): if nonempty then modify the plots of rejected
%   patches: rpshow = 2 causes rejected patches to be colored according to their
%   reason for rejection; rpshow = 1 is a noop; rpshow = 0 causes the plots of
%   rejected patches to be deleted.
%
%   Option 'ff' (default []): null or a handle to a function with the signature
%   p = ff(xx,yy,zz,r,opts) that calls patchfit on (xx,yy,zz).  r is the pick
%   radius, which may be used to initialize the patch boundary; opts are
%   additional options to pass to patchfit.
%
%   Option 'ssmethod' (default 'kdtree'): samplesearcher() search method
%
%   Option 'ssk' (default 5000): maximum number of neighbors for
%   samplesearcher()
%
%   Option 'ssr' (default 5e-2): search radius for samplesearcher()
%
%   Option 'adjtri' (default []): organized triangle mesh in the adjacency
%   format described in samplemesh.m.  adjtri is required for any of the BFS
%   neighborhood methods; if given its dimensions must match the xyz data, if
%   not given it will be internally generated.
%
%   Option 'triarea' (default []): area of adjtri mesh, empty to auto-compute
%
%   Option 'mink, maxk' (default -1.5/ssr, 1.5/ssr): minimum and maximum
%   curvature for kept patches, nan to disable check
%
%   Option 'maxr' (default 0.2*ssr): maximum residual for kept patches,
%   nonpositive or nan to disable check
%
%   Option 'rtype' (default 'newton'): residual type for patchresidual()
%
%   Option 'minc' (default 0.75): minimum coverage percent for kept patches,
%   nonpostive or nan to disable check
%
%   Option 'wc' (default 0): patchcoverage() cell side length if positive
%
%   Option 'nc' (default 25): patchcoverage() desired number of cells if
%   positive; exactly one of wc, nc must be positive.
%
%   Option 'maxs' (default 1000): maximum number of seed points to try, nan
%   or nonpositive to disable check
%
%   Option 'maxp' (default 1000): maximum number of patches to keep, nan or
%   nonpositive to disable check
%
%   Option 'mina' (default 0): minimum sum area of kept patches for termination,
%   nan or nonpositive to disable check
%
%   Option 'minar' (default 0.9): minimum ratio of sum area of kept patches
%   to mesh area, nan or nonpositive to disable check
%
%   Option 'maxt' (default 1000): maximum runtime in seconds, nan or
%   nonpositive to disable check
%
%   Option 'dbg' (default 0): may have any nonnegative value. dbg=0 disables
%   debug.  Larger values enable successively more verbose debug.
%     dbg = 1: all messages
%     dbg = 2: all messages+pauses
%     dbg = 3: non-iterative messages only
%
% Copyright (C) 2013 Marsette A. Vona and Dimitrios Kanoulas

tstart = tic();

minn = 10; % minimum neighborhood size to call patchfit

% patchfit override options
%pfo = {'dbg',6};
pfo = {};

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
cxx = 0; cyy = 0; czz = 0; cxy = 0; cyz = 0; cxz = 0;
nosplot = 0; nopplot = 0; rpshow = 0; ff = [];
ssmethod = 'kdtree'; ssk = 5000; ssr = 5e-2;
adjtri = []; triarea = [];
mink = -1.5/ssr; maxk = 1.5/ssr;
maxr = 0.2*ssr; rtype = 'newton';
minc = 0.75; wc = 0.5*ssr; nc = 0; maxs = 1000; maxp = 1000;
mina = 0; minar = 0.9; maxt = 1000;
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'cxx'; cxx = v; case 'cyy'; cyy = v; case 'czz'; czz = v;
      case 'cxy'; cxy = v; case 'cyz'; cyz = v; case 'cxz'; cxz = v;
      case 'nosplot'; nosplot = v; case 'nopplot'; nopplot = v;
      case 'rpshow'; rpshow = v; case 'ff'; ff = v;
      case 'ssmethod'; ssmethod = v; case 'ssk'; ssk = v; case 'ssr'; ssr = v;
      case 'adjtri'; adjtri = v; case 'triarea'; triarea = v;
      case 'mink'; mink = v; case 'maxk'; maxk = v;
      case 'maxr'; maxr = v; case 'rtype'; rtype = v;
      case 'minc'; minc = v; case 'wc'; wc = v; case 'nc'; nc = v;
      case 'maxs'; maxs = v; case 'maxp'; maxp = v;
      case 'mina'; mina = v; case 'minar'; minar = v; case 'maxt'; maxt = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

ccc = {cxx,cyy,czz,cxy,cyz,cxz}; ccs = cellfun(@isscalar,ccc);
if (any(ccs))
  if (~all(ccs)); error('all covars must be scalar or MxN'); end
else
  if (any(cellfun(@(c)(~isequal(size(c),size(x))),ccc)))
    error('all covars must be scalar or MxN');
  end
end

if (~nosplot); h = sampleplot('x',x,'y',y,'z',z); hold('on'); drawnow(); end

  function p = dofit(xx,yy,zz,r,opts) % default fit function
    if (~isempty(r))
      ii = find(strcmpi(opts,'db'),1,'last');
      if (~isempty(ii)); opts{ii+1} = r; else opts = {opts{:},'db',r}; end
    end
    p = patchfit(xx,yy,zz,opts{:},pfo{:})
  end % dofit

if (isempty(ff)); ff = @dofit; end

if (dbg>0); dbgmsg = @(varargin)(fprintf(varargin{:}));
else dbgmsg = @(varargin)(false); end

if ((dbg>0)&&(dbg<3)); itdbgmsg = @(varargin)(fprintf(varargin{:}));
else itdbgmsg = @(varargin)(false); end

% cumulative timing
tmesh = 0; tsearcher = 0; tsearch = 0; tfit = 0; tres = 0; tcvg = 0; tarea = 0;
npf = 0; npr = 0; npc = 0;

% generate mesh and/or mesh area
if ((~isempty(minar)&&(minar>0)&&...
     (isempty(triarea)||isnan(triarea)||(triarea<=0)))||...
    (isempty(adjtri)&&strncmpi('bfs',ssmethod,3)))
  ts = tic();
  [adjtri,triarea] = samplemesh('x',x,'y',y,'z',z,'adjtri',adjtri);
  tmesh = toc(ts);
end

% create neighborhood searcher
ts = tic();
searcher = samplesearcher(x,y,z,ssk,'method',ssmethod,'adjtri',adjtri);
tsearcher = toc(ts);

sz = size(x); nd = sz(1)*sz(2); % data size

ns = 0; % number of seeds generated
nkp = 0; nrp = 0; % number of kept and rejected patches
area = 0; % total area of kept patches

lkp = 1000; if (maxp>0); lkp = maxp; end
kpp = cell(lkp,1); rpp = cell(1000,1); % vectors of kept and rejected patches
  function vv = grow(v); vv = cell(2*length(v),1); vv(1:length(v)) = v; end

if (rpshow==2)
  dbgmsg('rejection colors: yel-curvature; mag-residual; grn-coverage\n');
end

% generate seeds until some termination condition holds
while 1

  if ((maxt>0)&&(toc(tstart)>maxt))
    dbgmsg('terminating, exceded %g seconds\n',maxt);
    break;
  end

  if ((maxs>0)&&(ns>=maxs))
    dbgmsg('terminating, generated maximum seeds %g\n',maxs);
    break;
  end

  % generate and validate a new seed
  seed = randi(nd); ns = ns+1;
  if (isnan(x(seed))||isnan(y(seed))||isnan(z(seed))); continue; end

  % fetch and validate neighborhood
  ts = tic();
  nbrs = searcher(seed,ssr);
  tsearch = tsearch+toc(ts);

  if (length(nbrs)<minn); continue; end

  xx = x(nbrs); yy = y(nbrs); zz = z(nbrs);

  % get neighborhood covars
  if (~isscalar(cxx))
    ccxx = cxx(nbrs); ccyy = cyy(nbrs); cczz = czz(nbrs);
    ccxy = cxy(nbrs); ccyz = cyz(nbrs); ccxz = cxz(nbrs);
  else
    ccxx = cxx; ccyy = cyy; cczz = czz;
    ccxy = cxy; ccyz = cyz; ccxz = cxz;
  end

  % fit the patch
  p = ff(xx,yy,zz,ssr,...
         {'cxx',ccxx,'cyy',ccyy,'czz',cczz,'cxy',ccxy,'cyz',ccyz,'cxz',ccxz});
  npf = npf+1;
  if (isempty(p)); continue; end
  tfit = tfit+p.fitsec;

  itdbgmsg('fit candidate patch to %d neighbors\n',length(nbrs));

  % plot it
  if (~nopplot)
    if (isfield(p,'hp')&&(~isempty(p.hp))); delete(p.hp); end
    p.hp = patchplot(p);
    drawnow(); % expensive
  end

  keep = 1; color = [];

  if ((min(p.k)<mink)||(max(p.k)>maxk))
    keep = 0; color = 'y';
    itdbgmsg('rejecting patch due to curvature %g<%g or %g>%g\n',...
             min(p.k),mink,max(p.k),maxk);
  end

  if (keep&&(maxr>0)) % check residual

    ts = tic();
    res = patchresidual(p,'x',xx,'y',yy,'z',zz,'type',rtype);
    npr = npr+1;
    tres = tres+toc(ts);

    if (res>maxr)
      keep = 0; color = 'm';
      itdbgmsg('rejecting patch due to residual %g>%g\n',res,maxr);
    end

  end % check residual

  if (keep&&(minc>0)) % check coverage

    ts = tic();
    [cvg, ~, cp, cm] = patchcoverage(p,'x',xx,'y',yy,'z',zz,'wc',wc,'nc',nc);
    %'dbg',(dbg>0)&&(dbg<3));
    npc = npc+1;
    tcvg = tcvg+toc(ts);

    if ((dbg==2)&&~isempty(p.hp))
      p.cvgp = cp; p.cvgm = cm;
      hp = patchplot(p,'df',0,'da',0,'dg',0,'db',0,'dc','g');
      hc = findobj(get(hp,'Children'),'flat','Tag','coverage');
      set(hc,'Parent',p.hp);
      delete(hp);
    else hc = []; end

    if (cvg<minc)
      keep = 0; color = 'g';
      itdbgmsg('rejecting patch due to coverage %g<%g\n',cvg,minc);
    end

  end % check coverage

  if (keep)

    nkp = nkp+1; kpp{nkp} = p;

    ts = tic();
    p = patchchk(p,'ke',1,'ga',1);
    tarea = toc(ts);

    area = area+p.ta;

    itdbgmsg('keeper: res %g, cvg %g, area %g\n',res,cvg,p.ta);
    itdbgmsg('totarea %g, triarea %g\n',area,triarea);

    if ((isnan(maxp)||(maxp<=0))&&(nkp==length(kpp))); kpp = grow(kpp); end

  else % reject

    nrp = nrp+1; rpp{nrp} = p;

    if (nrp==length(rpp)); rpp = grow(rpp); end

    if (isfield(p,'hp')&&(~isempty(p.hp)))
      switch (rpshow)
        case 0; delete(p.hp); p.hp = [];
        case 2; 
          set(findobj(get(p.hp,'Children'),'flat','Tag','boundary'),...
              'Color',color);
          ll = get(findobj(get(p.hp,'Children'),'flat','Tag','gridlines'),...
                   'Children');
          for i=1:length(ll); set(ll(i),'Color',color); end
          drawnow(); % expensive
      end
    end

  end % keep or reject

  if (dbg==2)
    es = input('autoseg eval (q <enter> quits, <enter> continues):','s');
    if (~isempty(hc)); delete(hc); hc = []; end
    if (strcmpi(es,'q')); break; else evalin('caller',es); end
  end

  if ((maxp>0)&&(nkp>=maxp))
    dbgmsg('terminating, generated maximum kept patches %g\n',maxp);
    break;
  end

  if ((mina>0)&&(area>=mina))
    dbgmsg('terminating, covered area %g>=%g\n',area,mina);
    break;
  end

  if ((minar>0)&&(area>=(minar*triarea)))
    dbgmsg('terminating, covered %g>=%g of total mesh area %g\n',...
           area/triarea,minar,triarea);
    break;
  end

end % generate seeds

kp = kpp(1:nkp);
if (nargout>1); rp = rpp(1:nrp); end

dbgmsg(['autoseg: %gs; %g seeds, %g patches kept, %g rejected\n',...
        '   mesh: %gs, searcher: %gs, search: %gs\n'...
        '   area: %g, mesh area: %g\n'...
        '    fit: %gs total, %gs/patch (%g patches)\n'...
        '    res: %gs total, %gs/patch (%g patches)\n'...
        '    cvg: %gs total, %gs/patch (%g patches)\n'],...
       toc(tstart),ns,nkp,nrp,tmesh,tsearcher,tsearch,area,tarea,...
       tfit,tfit/npf,npf,tres,tres/npr,npr,tcvg,tcvg/npc,npc);

end
