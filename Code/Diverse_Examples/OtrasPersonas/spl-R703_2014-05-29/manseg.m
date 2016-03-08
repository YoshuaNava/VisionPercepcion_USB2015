function pp = manseg(x,y,z,varargin)
% pp = manseg(x,y,z) manually segments a dataset into a set of fit patches
%
%   Arguments x, y, z must be real matrices of the same size MxN giving the
%   point sample coordinates in world frame. Variances and covariances may
%   optionally be specified for each point, see OPTIONS below.
%
%   The data is plotted with sampleplot() into the curent axes, or a new figure
%   window is opened if none is current, unless the option 'nosplot' is given.
%   Neighborhood selection is handled by samplepick(), see the 'dsel' option.
%   Patches are fit with patchfit(), see the 'ff' option.
%
%   The return is Px1 where P are the number of fitted patches "kept" by the
%   user.  Each has the optional field hp set to the corresponding
%   patchplot() graphics handle, if any.
%
%   Also see autoseg.m.
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
%   Option 'dsel' (default []): Indicates existing dataset for picking, either
%   empty or 'first' (uses first found) or a scalar handle to a scatter3 object
%   tagged 'samples' that is a child of a hggroup returned from sampleplot().
%   Must be [] unless 'nosplot' is also set.
%
%   Option 'ff' (default []): null or a handle to a function with the signature
%   p = ff(xx,yy,zz,r,opts) that calls patchfit on (xx,yy,zz).  r is the pick
%   radius, which may be used to initialize the patch boundary; opts are
%   additional options to pass to patchfit.
%
%   Option 'adjtri' (default []): organized triangle mesh in the adjacency
%   format described in samplemesh.m.  adjtri is required for any of the BFS
%   pick methods and its dimensions must match the xyz data.
%
% Copyright (C) 2013 Marsette A. Vona

minn = 10; % minimum neighborhood size to call patchfit

% patchfit override options
%pfo = {'dbg',6};
pfo = {};

% patchcoverage options
pco = {'wc',0,'nc',25};

% pick methods and samplesearcher options
searcher = []; searchmax = 5000; seed = []; seedc = []; searchn = [];
pkmthds = {'cylinder', 'kdtree', 'exhaustive', 'bfsdd', 'bfsmd'};
pkmthd = 1; % current pick method index

  function nm = nextpkmthd()
  nm = pkmthd+1; if (nm>length(pkmthds)); nm = 1; end
  end

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
cxx = 0; cyy = 0; czz = 0; cxy = 0; cyz = 0; cxz = 0;
nosplot = 0; nopplot = 0;
dsel = []; ff = []; adjtri = [];

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'cxx'; cxx = v; case 'cyy'; cyy = v; case 'czz'; czz = v;
      case 'cxy'; cxy = v; case 'cyz'; cyz = v; case 'cxz'; cxz = v;
      case 'nosplot'; nosplot = v; case 'nopplot'; nopplot = v;
      case 'dsel'; dsel = v; case 'ff'; ff = v; case 'adjtri'; adjtri = v;
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

if (ischar(dsel)&&strcmpi('first',dsel)); dsel = []; end
if (~isempty(dsel)&&~(ishandle(dsel)&&isscalar(dsel)))
  error('dsel must be empty, the string first, or a scalar handle');
end

if (~nosplot)
  if (~isempty(dsel)); error('dsel must be empty unless nosplot is set'); end
  h = sampleplot('x',x,'y',y,'z',z); hold('on'); drawnow();
  dsel = findobj(get(h,'Children'),'flat','Tag','samples');
end

  function p = dofit(xx,yy,zz,r,opts) % default fit function
    
    if (~isempty(r)) % set default patch boundary radius
      ii = find(strcmpi(opts,'db'),1,'last');
      if (~isempty(ii)); opts{ii+1} = r; else opts = {opts{:},'db',r}; end
    end
    
    p = patchfit(xx,yy,zz,opts{:},pfo{:})
    
  end

if (isempty(ff)); ff = @dofit; end

  function ii = pick(r,c,d,s,dbg) % pick w/ searcher or cylinder select

  if (nargin<5); dbg = 0; end
  
  if (strcmpi(pkmthds{pkmthd},'cylinder')) % cylinder

    ts = tic();
    searcher = [];
    ii = inrball(r,c(1:2),d(1:2,:)); 
    if (dbg); fprintf('cylinder select: %gs\n',toc(ts)); end

  else % search

    % cache seed and searcher as long as size of dataset doesn't change
    %
    % this method is both a callback from samplepick for visual feedback while
    % dragging with a possibly decimated dataset and a level of indirection for
    % sel() below which operates on the full dataset

    [~,n] = size(d);
    if (isempty(seed)||~isequal(seedc,c(1:2))||(searchn~=n)) % cache invalid
      [~,seed] = min(sqrt(sum((d(1:2,:)-repmat(c(1:2),1,n)).^2)));
      if (~isscalar(seed)); seed = seed(1); end
    end

    if (isempty(searcher)||(searchn~=n)) % cache invalid
      if (isequal(s,size(x))) % use full dataset
        xx = x; yy = y; zz = z; at = adjtri;
      else % use display dataset
        xx = reshape(d(1,:),s); yy = reshape(d(2,:),s); zz = reshape(d(3,:),s);
        if (strncmpi('bfs',pkmthds{pkmthd},3))
          at = samplemesh('x',xx,'y',yy,'z',zz,'dbg',dbg);
        else at = []; end
      end
      searcher = samplesearcher(xx,yy,zz,searchmax,...
                                'method',pkmthds{pkmthd},'adjtri',at,'dbg',dbg);
    end

    seedc = c(1:2); searchn = n;

    ii = searcher(seed,r,dbg);

  end % search
  end % pick

msx = []; msy = []; msz = []; % currently selected data
msp = []; % curently fitted patch
mskp = []; % set of kept patches
nsf = 1; % index for generating neighborhood save filenames

  function [xx,yy,zz,ccxx,ccyy,cczz,ccxy,ccyz,ccxz] = sel(cp,pr,rr,tt,ii)
  % select helper
  % cp is the center 'front' point of the pick in cam frame, empty if no pick
  % pr is the pick radius
  % rr{1}, tt{1} give the local-to-cam transform for the [x,y,z] data
  % ii is either empty or ii{1} are the indices of the picked data

  if (isempty(ii)) % cannot use ii from samplepick when gfx do not show all data
    if (~isempty(cp)) % do our own pick
      cxyz = [x(:),y(:),z(:)]'; [~,n] = size(cxyz);
      cxyz = rr{1}*cxyz+repmat(tt{1},1,n); % local to camera
      ii = pick(pr,cp,cxyz,size(x),1);
    else ii = []; end % no pick
  else ii = ii{1}; end % use indices from samplepick

  xx = x(ii); yy = y(ii); zz = z(ii);

  if (nargout > 3)
    if (~isscalar(cxx))
      ccxx = cxx(ii); ccyy = cyy(ii); cczz = czz(ii);
      ccxy = cxy(ii); ccyz = cyz(ii); ccxz = cxz(ii);
    else
      ccxx = cxx; ccyy = cyy; cczz = czz; ccxy = cxy; ccyz = cyz; ccxz = cxz;
    end
  end
  end % sel()

  function mshook(key,cr,ct,cp,pr,gg,rr,tt,ii) % samplepick() key hook
 
  switch (key)
    
    case 'f'; % (re)fit patch

      % forget any transient patch
      if (~isempty(msp)); delete(msp.hp); msp = []; end
        
      [msx,msy,msz,mscxx,mscyy,msczz,mscxy,mscyz,mscxz] = sel(cp,pr,rr,tt,ii);
      msnd = length(msx);
        
      fprintf('%d samples selected\n',msnd);
        
      if (msnd>minn)
        fprintf('(re)fitting\n');
        msp = ff(msx,msy,msz,pr,...
                 {'cxx',mscxx,'cyy',mscyy,'czz',msczz,...
                  'cxy',mscxy,'cyz',mscyz,'cxz',mscxz});
        if (~nopplot&&~isempty(msp))
          if (isfield(msp,'hp')&&(~isempty(msp.hp))); delete(msp.hp); end
          msp.hp = patchplot(msp); drawnow();
        end
      else fprintf('%d samples required for fitting, got %d\n',minn,msnd); end
      
    case 'k'; % keep patch
      
      if (~isempty(msp))

        fprintf('keeping last patch\n');

        % delete any coverage data and gfx
        if (isfield(msp,'cvgp')); msp = rmfield(msp,'cvgp'); end
        if (isfield(msp,'cvgm')); msp = rmfield(msp,'cvgm'); end
        delete(findobj(get(msp.hp,'Children'),'flat','Tag','coverage'));

        mskp = [mskp msp]; msp = [];

      else fprintf('no patch to keep'); end
      
    case 'u'; % unkeep patch
      
      fprintf('unkeeping last patch, if any\n');

      % forget about transient patch, if any, else forget last kept patch
      if (~isempty(msp)); delete(msp.hp); msp = [];
      elseif (~isempty(mskp)); delete(mskp(end).hp); mskp = mskp(1:end-1); end
      
    case 'c'; % clear patches
      
      fprintf('clearing all patches\n');
      if (~isempty(msp)); delete(msp.hp); msp = []; end
      for p=mskp(:)'; delete(p.hp); end; mskp = [];
      
    case 's'; % save neighborhood
      
      [msx,msy,msz] = sel(cp,pr,rr,tt,ii); msnd = length(msx);
      if (msnd>0)
        while (1) % make unique filename
          sfn = sprintf('mspatch%d.ply',nsf); nsf = nsf+1;
          if (~exist(sfn,'file')); break; end
        end
        % do not apply datafn() to sfn, always save in cwd
        fprintf('saving %d samples to %s\n',msnd,sfn);
        samplesave(msx,msy,msz,sfn);
      else fprintf('no samples currently selected\n'); end
      
    case 'i'; % info

      if (~isempty(msp)) % give info on transient patch

        fprintf('patch parameters:\n'); patchprint(msp);

        rrinv = rexp(-msp.r); msnd = length(msx);
        xyzw = [msx(:),msy(:),msz(:)]*rrinv'+repmat((-rrinv*msp.c(:))',msnd,1);
        msxl = xyzw(:,1); msyl = xyzw(:,2); mszl = xyzw(:,3);

        for t={'exact','newton','vert','taubin1','taubin2'}
          [res, sec] = patchresidual(msp,'x',msxl,'y',msyl,'z',mszl,...
                                     'src','xyzl','type',t{1});
          fprintf('%10s residual: %g (%g sec)\n',t{1},res,sec);
        end

        [pct, sec, cp, cm] = patchcoverage(msp,'x',msxl,'y',msyl,pco{:});

        fprintf('coverage: %g (%g sec)\n',pct,sec);
        fprintf('  wc=%g, nc=%g, ng=%g, eo=%g, ba=%g, nd=%g, nei=%g\n',...
                cp(1),cp(2),cp(3),cp(4),cp(5),cp(6),cp(15));
        fprintf('  lc=%g, rc=%g, tc=%g, bc=%g, ti=%g, to=%g, zi=%g, zo=%g\n',...
                cp(7),cp(8),cp(9),cp(10),cp(11),cp(12),cp(13),cp(14));

        if (~isempty(msp.hp))
          msp.cvgp = cp; msp.cvgm = cm;
          hp = patchplot(msp,'df',0,'da',0,'dg',0,'db',0,'dc','g');
          hc = findobj(get(hp,'Children'),'flat','Tag','coverage');
          set(hc,'Parent',msp.hp); delete(hp);
        end

      else % give info on current selection
        [msx,msy,msz] = sel(cp,pr,rr,tt,ii); msnd = length(msx);
        fprintf('%d samples selected\n',msnd);
      end

    case 'm'; % method

      % skip bfs search methods when adjtri not available
      while(isempty(adjtri)&&strncmpi('bfs',pkmthds{nextpkmthd()},3))
        pkmthd = nextpkmthd();
      end

      pkmthd = nextpkmthd(); searcher = [];

      fprintf('pick method: %s', pkmthds{pkmthd});
      if (~strcmpi(pkmthds{pkmthd},'cylinder'));
        fprintf(', searchmax=%d',searchmax);
      end
      fprintf('\n');
      
    case {'q','escape','return'}; % quit
      
      fprintf('manual segmentation done\n');

      % delete any transient fitted patch, keepers are in kp
      if (~isempty(msp)); delete(msp.hp); msp = []; end
      
    otherwise; % ignore
  end % switch(key)
  end % mshook

% Cover mshook to ignore the picked indices if the plotted data looks like
% a subset of the actual data.  In this case samplepick() will pick in the
% visible data for graphical feedback but the sel() function above will
% re-do the picks in the full dataset for patch fitting.

% First we need to resolve a handle to the visible dataset.
if (isempty(dsel))
  hd = findobj(gca(),'Tag','samples');
  if (length(hd)>1); hd = hd(1); end
else hd = dsel; end

if (numel(x)==length(get(hd,'XData')))
  msh = @mshook;
else % cover mshook to not request the final arg ii from samplepick()
  msh = @(key,cr,ct,cp,pr,gg,rr,tt)(mshook(key,cr,ct,cp,pr,gg,rr,tt,{}));
  fprintf('not displaying full dataset, disabled optimized pick\n');
end

fprintf('f=fit, u/k=(un)keep patch, c=clear, s=save, i=info, m=method\n');

samplepick(msh,dsel,@pick);

pp = mskp;

end
