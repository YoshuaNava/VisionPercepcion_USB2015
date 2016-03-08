function p = patchfit(x, y, z, varargin)
% patchfit(x, y, z) fits a patch to point samples
%
%   Arguments x, y, z must be real matrices of the same size MxN giving the
%   point sample coordinates in world frame. Variances and covariances may
%   optionally be specified for each point, see OPTIONS below. If any
%   coordinate, variance, or covariance for a point is NaN, infinite, or
%   complex, that point is ignored. If any covariance matrix for a non-ignored
%   point is not positive semi-definite an error will be raised.
%
%   The fitted surface and boundary types are controlled by the options
%   'st' and 'bt'.
%
%   Output p is the patch with fitted primary fields (see patchchk()).  The
%   following auxiliary fields are also set:
%
%   p.residual - residual of surface fit, see option 'dores'
%
%   p.coverage - patchcoverage() percentage if run, else [], see option 'docvg'
%
%   p.bcp - fitted boundary containment probability
%
%   p.fitsn, p.fitbn - number of fitted surface and boundary samples
%
%   p.hp - handle to plotted patch, if dbg gfx enabled (see 'dbg' option below)
%
%   p.fitsec - total fit time in seconds
%
%   p.ressec - patchresidual() time in seconds if called, see option 'dores'
%
%   p.covsec - patchcoverage() time in seconds if called, see option 'docvg'
%
%   p.covar - [] unless option 'docovar' is set.  Covariance matrix of the patch
%   parameters :
%
%   dx, dy, kx, ky, rx, ry, rz, cx, cy, cz (elliptic/hyperbolic paraboloid)
%
%   dx, dy,      k, rx, ry, rz, cx, cy, cz (cylindric paraboloid, circ cylinder)
%
%   d,           k, rx, ry,     cx, cy, cz (circular paraboloid, sphere)
%
%   dx, dy,         rx, ry, rz, cx, cy, cz (plane, ellipse/aarect)
%
%   d,              rx, ry,     cx, cy, cz (plane, circle)
%
%   d1, d2, d3, d4, gamma, rx, ry, rz, cx, cy, cz (plane, convex quad)
%
%   p.fitsd - the actual fitted datapoints in world frame as an Nx3 matrix if
%   the 'rsd' option is set, else [].  Note that the actual fitted datapoints
%   may not match the input: they may coerced to double, subsampled, cleaned of
%   nan/inf/complex, and vectorized, depending on requested options.
%
%   p.fitbd - the actual datapoints used for boundary fitting, projected to the
%   local frame xy plane, as a Px3 matrix if the 'rbd' option is set, else [].
%
%   Note: if dbg is enabled then it is possible for p to be returned as [] or
%   for any or all fields to be missing on the returned patch because in some
%   debug modes the user can quit the fitting process.
%
%   The fitting algorithm is described in the paper "Curved Surface Contact
%   Patches with Quantified Uncertainty" by Marsette Vona and Dimitrios
%   Kanoulas, IROS 2011.
%
%   OPTIONS
%
%   Options 'cxx', 'cyy', 'czz', 'cxy', 'cyz', 'cxz' (default 0):
%   covariance of the data. May each either be scalar or MxN.
%
%   Option 'st' (default 'a'): general type of fitted surface.  Must be 'a'
%   (general paraboloid), 'p' (plane), 's' (sphere), or 'c' (circular
%   cylinder).
%
%   Option 'bt' (default 'e' if st='p', [] otherwise): sets the fitted
%   boundary type if 'st' is 'p' (plane). Otherwise must be [].
%
%   Option 'bcp' (default 0.95): boundary containment probability in (0,1].
%
%   Option 'ssmax' (default 0): if positive then this limits the maximum
%   number of data samples used for surface fitting.
%
%   Option 'bsmax' (default 0): if positive then this limits the maximum
%   number of data samples used for boundary fitting (if bsmax is set
%   greater than ssmax it is clamped to ssmax).
%
%   Option 'noise' (default 0): if positive then this gives the standard
%   deviation of Gaussian white noise for input data perturbation.
%
%   Option 'ktol' (default 0.1): curvature comparison tolerance, must be
%   nonnegative.
%
%   Option 'ppss' (default 0.2): samplesize for dbg patchplot
%
%   Option 'ppas' (default 1.5): axes scale for dbg patchplot
%
%   Option 'db' (default 1): default boundary length parameter(s). May have
%   any length from scalar up; if more parameters are needed the last given
%   one is copied, except db(5), if not given, is always implied as pi/4.
%
%   Option 'fb' (default 1): whether to fit the boundary curve; if not, it
%   is left at the value initialized from 'db'.
%
%   Option 'ccon' (default 1): constrain the center point of fitted paraboloid
%   patches to the line through the centroid of the data in the direction of the
%   normal of a plane fit to the data.  This behavior is implied for
%   non-paraboloid patches.
%
%   Option 'rsd' (default 0): whether to set p.fitsd (see above) on return
%
%   Option 'rbd' (default 0): whether to set p.fitbd (see above) on return
%
%   Option 'numj' (default 0): whether to always use numeric jacobians
%   instead of analytic
%
%   Option 'usemo' (default 1): whether to use memoization of fit objective
%   function and its derivatives if possible (memoization is not compatible
%   with numeric Jacobians)
%
%   Option 'dores' (default 0): either 0 or 'alg' to use the internal algebraic
%   residual, or one of the types recongnized by patchresidual(), which will be
%   called with the data used for surface fitting.  Does not affect the residual
%   type used during the fit, which is always algebraic.
%
%   Option 'docvg' (default 0): whether to call patchresidual() on the fitted
%   patch with the data used for boundary fitting.  Either scalar or cell
%   array to give the coverage options.
%   
%   Option 'docovar' (default 0): whether to calculate p.covar
%
%   Option 'dbg' (default 0): may have any nonnegative value. dbg=0 disables
%   debug. Larger values enable successively more verbose debug.
%     dbg = 1: messages only
%     dbg = 2: messages+pauses+graphics
%     dbg = 3: messages+pauses+graphics+covars
%     dbg = 4: messages+graphics
%     dbg = 5: graphics+covars
%     dbg = 6: graphics only
%     dbg = 7: timing messages only
%
% Copyright (C) 2013 Marsette Vona and Dimitrios Kanoulas

tstart = tic();

% constants
xh = [1;0;0]; yh = [0;1;0]; zh = [0;0;1]; % unit basis vectors
i33 = eye(3,3); z33 = zeros(3,3); % common matrices

% dbg controls
prec = 3; printonsetp = 0; enwlmdbg = 1; drawsubtitles = 0;

spmax = 10000; cpmax = 100; % max samples and covars in dbg gfx

% sample, covariance, and patch plot options
sampleplotopts = {'subsample',spmax};
covarplotopts = {'p',0.99,'de',2,'da',0,'ns',5,'subsample',cpmax};
patchplotopts = {'ss',0.2,'gd',2,'aw',1,'as',1.5,...
                 'bw',2,'bc','r','gw',2,'gc','r','df',0,'fb',1};
%patchplotopts = {'ss',0.2,'gd',2,'da',0,'as',1.5,...
%                 'bw',2,'bc','r','gw',1,'gc','r','df',0,'fb',1};

% numeric optimizer opts
%chknd = 0.5;
chknd = 0;
wlmopts = {'minrda',1e-4,'minrd',1e-3};
%wlmopts = {'minrda',1e-6,'minrd',1e-6};
wlmopts = {wlmopts{:},'dfdd','num','dfda','num','d2fdadd','num'};

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
cxx = 0; cyy = 0; czz = 0; cxy = 0; cyz = 0; cxz = 0;
st = 'a'; bt = []; bcp = 0.95; ssmax = 0; bsmax = 0; noise = 0;
ktol = 0.1; ccon = 1;
db = 1; fb = 1; ppss = 0.2; ppas = 1.5; rsd = 0; rbd = 0;
numj = 0; usemo = 1;
dores = 0; docvg = 0; docovar = 0;
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'cxx'; cxx = v; case 'cyy'; cyy = v; case 'czz'; czz = v;
      case 'cxy'; cxy = v; case 'cyz'; cyz = v; case 'cxz'; cxz = v;
      case 'st'; st = v; case 'bt'; bt = v; case 'bcp'; bcp = v;
      case 'ssmax'; ssmax = v; case 'bsmax'; bsmax = v;
      case 'noise'; noise = v;
      case 'ktol'; ktol = v;
      case 'db'; db = v; case 'fb'; fb = v;
      case 'ppss'; ppss = v; case 'ppas'; ppas = v;
      case 'ccon'; ccon = v;
      case 'rsd'; rsd = v; case 'rbd'; rbd = v;
      case 'numj'; numj = v; case 'usemo'; usemo = v;
      case 'dores'; dores = v; case 'docvg'; docvg = v;
      case 'docovar'; docovar = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name');
  end
end

% check option constraints
if ((bcp <= 0) || (bcp > 1)), error('bcp must be in (0,1]'); end
if (ktol < 0), error('curvature tolerance must be nonnegative'); end

switch (st)
  case {'a','s','c'}
    if (~isempty(bt)); error('bt must be empty if st~=p'); end
  case 'p';
    if (isempty(bt)), bt = 'e';
    elseif (~any(strcmpi(bt,{'c','e','r','q'})))
      error('unrecognized boundary type %s',bt);
    end
  otherwise; error('unrecognized surface type %s',st);
end

if (isempty(db)), db = 1; end
db = db(:); ndb = length(db);
if (ndb<4), db = [db;repmat(db(ndb),4-ndb,1)]; ndb = length(db); end
if (ndb<5), db = [db;pi/4]; ndb = 5; end

% configure dbg
patchplotopts{find(strcmpi('ss',patchplotopts),1)+1} = ppss;
patchplotopts{find(strcmpi('as',patchplotopts),1)+1} = ppas;

% print a mat
  function s = m2s(m); s = mat2str(m,prec); end

% print a basis
  function s = r2s(r)
  if (isvector(r)), r = rexp(r); end
  s = sprintf('lx=%s, ly=%s, lz=%s',m2s(r*xh),m2s(r*yh),m2s(r*zh));
  end

% debug options and option handling for wlm
wlmdbg = 0;
if (enwlmdbg)
  if ((dbg==1)||(dbg==2)); wlmdbg = 2; elseif (dbg==3); wlmdbg = 3;
  elseif ((dbg==4)||(dbg==7)), wlmdbg = 1; end
end
wlmopts = {wlmopts{:},'dbg',wlmdbg,'fa',[],'upd',[]};

% convenience to mutate wlmopts
  function setwlmopts(varargin)
  for i=1:(length(varargin)/2)
    n = varargin{2*i-1}; v = varargin{2*i};
    ni = find(strcmpi(n,wlmopts),1);
    if (ni) wlmopts{ni+1} = v; else wlmopts = {wlmopts{:},n,v}; end
  end
  end

% dbgmsg('msg %d\n',foo)
isdbgmsg = ((dbg>0)&&(dbg~=5)&&(dbg~=6)&&(dbg~=7));
if (isdbgmsg), dbgmsg = @(varargin)(fprintf(varargin{:}));
else dbgmsg = @(varargin)(false); end

% dbgpause(p,'msg %d\n',foo);  (prints patch)
% dbgpause([],'msg %d\n',foo);
  function o = dbgpause(p,varargin)
  if (~isempty(varargin)); dbgmsg(varargin{:}); end
  if (isdbgmsg&&(~isempty(p))); patchprint(p); end
  o = 1;
  if ((dbg==2)||(dbg==3))
    es = input('patchfit eval (q <enter> quits, <enter> continues):','s');
    if (strcmpi(es,'q')), o = 0; else evalin('caller',es); end
  end
  end

% set subtitle
  function sst(s); if (drawsubtitles), subtitle(s); end; end
  
isdbggfx = (dbg>1)&&(dbg<7);

p = []; % in case we quit early

% save the full neighborhood (in case of decimation)
%xTot = x; yTot = y; zTot= z;

% clean, subsample, noise, and vectorize input data
[x,y,z,cxx,cyy,czz,cxy,cyz,cxz] = ...
  samplecvt('x',x,'y',y,'z',z,...
            'cxx',cxx,'cyy',cyy,'czz',czz,'cxy',cxy,'cyz',cyz,'cxz',cxz,...
            'coerce','double','subsample',ssmax,'clean',1,'noise',noise,...
            'vectorize',1);

nd = length(x);
dbgmsg('%d data points after cleaning and subsampling\n',nd);

if (isdbggfx)
  sampleplot('x',x,'y',y,'z',z,sampleplotopts{:}); hold('on');
  sst('(patchfit sample points)');
  if (~dbgpause([],'sample points\n')), return; end
end

if ((dbg==3)||(dbg==5))
  h = covarplot('cxx',cxx,'cyy',cyy,'czz',czz,'cxy',cxy,'cyz',cyz,...
                'cxz',cxz,'cx',x,'cy',y,'cz',z,covarplotopts{:});
  sst('(sample point error ellipsoids)');
  if (~dbgpause([],'error ellipsoids\n')), return; end
  delete(h);
  sst([]);
end

if (~dbgpause([],'initial data\n')), return; end

% prepare input data and covariance matrices E(:,:,i)
xyz = [x,y,z];
exx = reshape(cxx,1,1,nd); eyy = reshape(cyy,1,1,nd);
ezz = reshape(czz,1,1,nd); exy = reshape(cxy,1,1,nd);
eyz = reshape(cyz,1,1,nd); exz = reshape(cxz,1,1,nd);
E = [exx, exy, exz; exy, eyy, eyz; exz, eyz, ezz];

if (rsd); p.rsd = [x,y,z]; end

% could check that covariance matrices are positive semi-definite here with
% something like cholcov(), but for now let's just check them when we use
% them, if any p'*S*p comes out negative we will error.

% sets patch curvature/curvatures, possibly inferring surface type
sip = 0; % surface type in play? (no, not initially)
  function o = setpk(p,k)
  if (sip) % infer paraboloid surface type
    if (length(k)==2)
      if (sign(k(1))~=sign(k(2))), p.s = 'h'; else p.s = 'e'; end
    elseif (length(k)==1), p.s = 'o';
    else p.s = 'p';
    end
  else % surface type not in play
    switch(p.s)
      case 'e';
          assert(length(k)==2);
          if (sign(k(1))~=sign(k(2))), [~,i]=min(abs(k)); k(i) = 0; end
      case 'h';
        assert(length(k)==2);
        if (sign(k(1))==sign(k(2))), [~,i]=min(abs(k)); k(i) = 0; end
      case {'y','c','o','s'};
        assert(length(k)>=1);
        if (length(k)==2), assert(k(1)==0); k = k(2); else k = k(1); end
      case 'p'; assert(all(k==0)); k = [];
    end
  end
  p.k = k;
  o = p;
  end

% set/update patch parameters given as name/value pairs
  function o = setp(p,varargin)
  for i=1:(length(varargin)/2);
    n = varargin{2*i-1}; v = varargin{2*i}; v = v(:); % always column vector
    if (~strcmpi(n,'k')), p = setfield(p,n,v); else p = setpk(p,v); end
  end
  o = p;
  if (printonsetp&&(dbg<5)), patchprint(o); end
  end

hp = []; % handle to plotted patch

% refresh patchplot if dbg gfx, but always pass through p
  function p = dorfpp(p,hp)
  patchplot(p,'refresh',hp,patchplotopts{:}); drawnow();
  end
if (isdbggfx), rfpp = @dorfpp; else rfpp = @(p,hp)(p); end

% generate an update hook for wlm iff dbg
  function u = wlmupd(upd)
    function uu(a,i,cc)
    if ((i==0)||cc), upd(a); end
    end
  if ((dbg>1)&&(dbg<7)), u = @uu; else u = []; end
  end

% initialize patch
p.s = 'p'; p.b = 'e'; p.d = db(1:2); p.k = [];
p.r = [0 0 0]'; p.c = [mean(x) mean(y) mean(z)]';
p.fitsn = nd; p.bcp = bcp; p.residual = inf; p.covar = [];
if (isdbggfx), hp = patchplot(p,patchplotopts{:}); p.hp = hp; end
sst('(default patch)');
if (~dbgpause(p,'initialized patch\n')), return; end

% callback functions for wlm
% (anon functions are slow)

% first some general forms for derivatives

% general form for (partial f)/(partial ql)
% j is the (nd x 3) Jacobian of the objective func wrt to data in local frame
% ql is the (nd x 3) data in local frame
% k3 is the 3x1 vector of curvature parameters
  function j = dfdql(ql,k3)
  j = 2*(ql*diag(k3)-repmat(zh',nd,1));
  end

% general form for (partial f)/(partial q)
% j is the (nd x 3) Jacobian of the objective func wrt to data in world frame
% rr is the 3x3 local to world rotation matrix
% dfdql is from dfdql()
  function j = dfdq(rr,dfdql); j = dfdql*rr'; end

% general form for (partial ql)/(partial r)
% j is the (nd x 3 x 3) Jacobian of the local frame data wrt rotation vector r
% qc is the (nd x 3) data in world frame minus the local frame origin c
% drr is the 3x3x3 derivative of rr with respect to the 3x1 rotation vector r
  function j = dqldr(qc,drr)
  j = cat(3, qc*drr(:,:,1), qc*drr(:,:,2), qc*drr(:,:,3));
  end

% general form for (partial f)/(partial a)
% j is the (nd x 9) Jacobian of the objective func wrt the parameters in order
%   k1 k2 k3 r1 r2 r3 c1 c2 c3
% ql is the (nd x 3) data in local frame
% rr is the 3x3 local to world rotation matrix
% dqldr is from dqldr()
% dfdql is from dfdql()
  function j = dfda(ql,rr,dfdql,dqldr)
  dfdk = ql.*ql;
  dfdr = [sum(dfdql.*dqldr(:,:,1), 2), ...
          sum(dfdql.*dqldr(:,:,2), 2), ...
          sum(dfdql.*dqldr(:,:,3), 2)];
  dfdc = -dfdql*rr';
  j = [dfdk, dfdr, dfdc];
  end

% general form for (partial partial f)/((partial a)(partial q))
% j is the (3 x 9 x nd) tensor Hessian of the objective function first wrt
%   the data and second wrt the parameters in order k1 k2 k3 r1 r2 r3 c1 c2 c3
% ql is the (nd x 3) data in local frame
% rr is the 3x3 local to world rotation matrix
% drr is the 3x3x3 derivative of rr with respect to the 3x1 rotation vector r
% dqldr is from dqldr()
  function j = d2fdadq(ql,k3,rr,drr,dqldr)
  K = diag(k3);
  ddk = 2*[reshape(rr(:,1)*ql(:,1)',3,1,nd), ...
           reshape(rr(:,2)*ql(:,2)',3,1,nd), ...
           reshape(rr(:,3)*ql(:,3)',3,1,nd)];
  qk = K*ql'-repmat(zh,1,nd);
  ddr = 2*[reshape(drr(:,:,1)*qk,3,1,nd), ...
           reshape(drr(:,:,2)*qk,3,1,nd), ...
           reshape(drr(:,:,3)*qk,3,1,nd)];
  ddr = ddr + 2*[reshape(rr*K*dqldr(:,:,1)',3,1,nd), ...
                 reshape(rr*K*dqldr(:,:,2)',3,1,nd), ...
                 reshape(rr*K*dqldr(:,:,3)',3,1,nd)];
  ddc = repmat(-2*rr*diag(k3)*rr',[1,1,nd]);
  j = [ddk, ddr, ddc];
  end

% memoization
enmo = 1; amo = []; omo = []; dfdqmo = []; dfdamo = []; d2fdadqmo = [];
% memos are not compatible with numeric derivatives
  function setenmo(wlmopts)
  if (~usemo); enmo = 0; return; end
  for i=1:(length(wlmopts)/2)
    n = wlmopts{2*i-1}; v = wlmopts{2*i};
    switch (n)
      case 'dfdd'; case 'dfda'; case 'd2fdadd';
        if (ischar(v) && strcmpi(v,'num')); enmo = 0; end
      case 'chknd'; if (v ~= 0); enmo = 0; end
    end
  end
  end

% objective function and derivatives for plane

% 5 parameter algebraic form for a plane (yes, redundant)
% a(1:2) is the orientation vector
% a(3:5) is the center point
  function o = plane(q,a)
  rr = rexp([a(1:2);0]); zl = rr(:,3); c = a(3:5);
  o = -2*(q-repmat(c',nd,1))*zl;
  end

% perp proj of c onto 5 param plane a
  function c = planec(a,c)
  rr = rexp([a(1:2);0]); zl = rr(:,3);
  c = c-(zl'*(c-a(3:5)))*zl;
  end

% objective function and derivatives for paraboloid

% 8 parameter algebraic form for paraboloid
% a(1:2) are the x, y curvatures
% a(3:5) is the orientation vector
% a(6:8) is the center point
  function o = parab(q,a)
  if (enmo && isequal(a,amo)); o = omo; return; end
  k3 = [a(1:2);0]; rv = a(3:5); rr = rexp(rv);
  if (ccon); c = plane_c+a(6)*plane_n; else c = a(6:8); end
  qc = q-repmat(c',nd,1); ql = qc*rr; 
  o = (ql.*ql)*k3-2*ql*zh;
  if (enmo)
    amo = a; omo = o; drr = drexp(rv);
    dfdqmo = parab_dfdq(a,ql,k3,rr);
    dfdamo = parab_dfda(a,ql,qc,k3,rr,drr);
    d2fdadqmo = parab_d2fdadq(a,ql,qc,k3,rr,drr);
  end
  end

  function j = parab_dfdq(a,varargin)
  if (enmo&&isempty(varargin)); j = dfdqmo; return; end
  if (isempty(varargin)||(ischar(varargin{1})&&strcmpi(varargin{1},'force')))
    k3 = [a(1:2);0]; rr = rexp(a(3:5)); q = xyz;
    if (ccon); c = plane_c+a(6)*plane_n; else c = a(6:8); end
    ql = (q-repmat(c',nd,1))*rr; 
  else ql = varargin{1}; k3 = varargin{2}; rr = varargin{3};
  end
  j = dfdq(rr,dfdql(ql,k3));
  end

  function j = parab_dfda(a,varargin)
  if (enmo&&isempty(varargin)); j = dfdamo; return; end
  if (isempty(varargin)||(ischar(varargin{1})&&strcmpi(varargin{1},'force')))
    k3 = [a(1:2);0]; rv = a(3:5); rr = rexp(rv); drr = drexp(rv); q = xyz;
    if (ccon); c = plane_c+a(6)*plane_n; else c = a(6:8); end
    qc = q-repmat(c',nd,1); ql = qc*rr; 
  else
    ql = varargin{1}; qc = varargin{2};
    k3 = varargin{3}; rr = varargin{4}; drr = varargin{5};
  end
  j = dfda(ql,rr,dfdql(ql,k3),dqldr(qc,drr));
  j = j(:,[1:2,4:9]);
  if (ccon); m = eye(8,6); m([6:8],6) = plane_n; j = j*m; end
  end

  function j = parab_d2fdadq(a,varargin)
  if (enmo&&isempty(varargin)); j = d2fdadqmo; return; end
  if (isempty(varargin)||(ischar(varargin{1})&&strcmpi(varargin{1},'force')))
    k3 = [a(1:2);0]; rv = a(3:5); rr = rexp(rv); drr = drexp(rv); q = xyz;
    if (ccon); c = plane_c+a(6)*plane_n; else c = a(6:8); end
    qc = q-repmat(c',nd,1); ql = qc*rr; 
  else
    ql = varargin{1}; qc = varargin{2};
    k3 = varargin{3}; rr = varargin{4}; drr = varargin{5};
  end
  j = d2fdadq(ql,k3,rr,drr,dqldr(qc,drr));
  j = j(:,[1:2,4:9],:);
  if (ccon)
    j = [j(:,[1:5],:), ...
         sum([plane_n(1)*j(:,6,:),plane_n(2)*j(:,7,:),plane_n(3)*j(:,8,:)],2)];
  end
  end

% objective function and derivatives for sphere

% generates a 4 parameter algebraic form for sphere
% zl is the local frame z axis basis vector, held fixed
% a(1) is the curvature
% a(2:4) is the apex point
  function f = sphere(zl)
    function o = ff(q,a)
    k3 = [a(1);a(1);a(1)]; c=a(2:4);
    ql = (q-repmat(c',nd,1)); o = (ql.*ql)*k3-2*ql*zl;
    end
  f = @ff;
  end

% objective function and derivatives for circ cyl

% get canonical local frame basis from ccyl params
  function [rr,xl] = cylrr(a,zl)
  rr=rexp([0;a(2:3)]); xl=rr*xh; yl=cross(zl,xl); ny = norm(yl);
  if (ny>1e-6), yl = yl/ny; rr=[xl yl cross(xl,yl)]; end
  end

% generates 6 parameter algebraic form for circular cylinder
% zl is the local frame z axis basis vector, held fixed
% a(1) is the curvature
% a(2:3) are a 2D orientation vector controlling the local frame x axis
% a(4:6) is a point on the apex of the cylinder
  function f = ccyl(zl)
    function r = ff(q,a)
    k3 = [0;a(1);a(1)]; rr = cylrr(a,zl); c=a(4:6);
    ql = (q-repmat(c',nd,1))*rr; r = (ql.*ql)*k3-2*ql*zh;
    end
  f = @ff;
  end

% generate function to canonicalize 2D or 3D rotation vector embedded in a
% ai are the indices in a
% ri are the indices in 3D r
  function f = fareparam(ai,ri)
    function a = ff(a)
    rv = [0 0 0]'; rv(ri) = a(ai); rv = rreparam(rv); a(ai) = rv(ri);
    end
  f = @ff;
  end

% STEP 1: fit plane by lls then by wlm
dbgmsg('fitting plane w/o input uncertainty by lls\n');

% solve for homogenous solution for zl (which is also the plane normal)
zl = lls([x-p.c(1),y-p.c(2),z-p.c(3)]);

% calculate p.r as in rcanon2, but p.r~=0 if t~=0
p.r = cross(zh,zl); ss = norm(p.r); cc = zh'*zl; t = atan2(ss,cc);
th = sqrt(sqrt(eps(1)))*10;
if (t>th), aa = t/ss; else aa = 6/(6-t^2); end
p.r = p.r*aa;

% The plane is now defined by p.c (a point on it) which was initialized
% above to the centroid of the data; and p.r --- the third column of
% rexp(p.r) is zl, the plane normal. Note that p.r is 3x1 but p.r(3)==0, 
% i.e. p.r is currently a 2D orientation vector, because an unbounded plane
% is rotationally symmetric about its normal.

rfpp(p,hp); sst('(initial fit plane w/o input uncertainty)');
if (~dbgpause(p,'fit plane c=%s\n%s\n', m2s(p.c), r2s(p.r))), return; end

% Save the zl and the centroid p.c, for patch center constraint's use.
plane_n = zl;
plane_c = p.c;

% Now refine plane fit by wlm iff not fitting general paraboloid.
%
% Consider the four possibilities for st:
%
% st=='a': We will shortly be fitting a general *assymetric* paraboloid by
% WLM in step 2. It's probably not worth it to further refine the patch 
% here by another relatively expensive WLM. The plane fit by LLS above will
% serve to initialize WLM to the correct region of parameter space (which 
% is important, because WLM will only find a local optimum), but all
% components of p.r and p.c (the only parameters we have estimated so far)
% will be replaced by the WLM in step 2.
%
% st=='p': The end goal is to fit a plane, so refine it here, and
% importantly, get its covariance matrix.  There will be no WLM in step 2.
% At the end of step 1 the plane will be defined by p.c and p.r; the former
% is a point on the plane and the latter gives the plane normal (the normal
% is the third column of rexp(p.r); even though p.r is 3x1 its third
% element is always zero, making it a 2D orientation vector). There is some
% subtlety here because an *unbounded* plane really has only 3DoF, not 
% 3+2=5. The extra two DoF are actually constrained by our desirement to
% keep p.c at the projection of the centroid of the data points onto the
% plane. See additional discussion inside the if block below. Also,
% remember there is a choice of boundary shapes for plane fitting, and some
% will break the rotational symmetry of the unbounded plane (all boundary
% shapes except circular will break the symmetry). This is handled later in
% step 9, where it may be necessary to extend p.r from 2D to 3D.
%
% st=='s': We are fitting a spherical patch. There will be a WLM in step 2, 
% but that will fit only an *unbounded* (i.e. complete) sphere. The 
% *orientation* of the patch will thus not come from that WLM. Instead, it 
% is determined by the plane normal calculated here in step 1. We thus need
% to do the best we can to estimate that normal, including the uncertainty 
% of the input data points E. The LLS done above did not consider that
% uncertainty, so we refine the plane fit with WLM now, even though we're 
% still going to do another WLM in step 2 to fit the sphere. In this case 
% p.c coming out of stage 1 is *not* going to matter because the WLM fit in
% stage 2 is going to determine p.c as the sphere apex ("pole") through an 
% axis parallel to the plane normal.
%
% st=='c': Fitting a circular cylindrical patch.  The logic is similar for
% the case st=='s': the WLM in step 2 is actually going to fit an 
% *unbounded* cylinder. The orientation of the final bounded cylindrical
% patch will come from a *combination* of the plane normal we calculate 
% here and the direction of the cylinder symmetry axis recovered in the WLM
% in step 2. In this case p.c again has some subtlety; similar to the case 
% for sphere fitting, it will be set in step 2 as *a* point on the apex of
% the cylinder through an axis parallel to the plane normal. But now that
% "apex" is not a single point but an unbounded line. Thus even after step 
% 2 p.c could "slide" arbitrarily along that apex line. This is finally
% resolved in step 6 by setting p.c as the projection of the data centroid
% *constrained to the cylinder axis*.
%
if (~strcmpi(st,'a'))
  
  % Fit plane by WLM.
  %
  % The current implementation uses a *redundant* parameterization here:
  % the first two elements of the parameter vector a are the first two
  % components of p.r (the third component is always zero, p.r is a 2D
  % orientation vector here), determining the plane normal (this part is
  % not redundant); the latter three elements of a are p.c, a point on the
  % plane (this is the redundant part: the point has two extra DoF to slide
  % around on the plane).
  %
  % TBD we should consider reworking this to be non-redundant, e.g. by
  % defining the center to be the point at some distance from the centroid 
  % of the data in the direction of the plane normal (the parameter vector 
  % a would then be 3x1, its last element would be said distance).
  %
  % For now the redundancy is compensated by two things: (1) the projection
  % function planec() is constantly adjusting p.c so that it stays at the
  % projection of the centroid of the data (its initial value coming in)
  % onto the plane (this is a particularly desirable choice of p.c, see
  % below); (2) the uncertainty propagation includes this projection step 
  % and thus should ignore the (expectably infinite) uncertainty in the 
  % components of p.c perpendicular to the plane normal zl (c.f. dcdc in 
  % the uncertainty propagation).
  %
  % While it might seem we are free at this stage to pick any p.c that lies
  % in the plane, the bigger picture is that we really want to pick a point
  % that is well centered in the data, because all our various planar patch
  % boundary shapes (ellipse, circle, aa rect, convex quad) are
  % origin-centered. This explains why it is actually an advantage to take 
  % p.c as the projection of the centroid of the data onto the plane. The 
  % current implementation is effectively forcing this as a constraint; the
  % proposal in the TBD paragraph above would instead maintain this as an
  % invariant by construction.
  
  dbgmsg('fitting plane with input uncertainty by wlm\n');
  a = [p.r(1:2);p.c]; ctr = p.c;
  upd = @(a)(rfpp(setp(p,'r',[a(1:2);0],'c',planec(a,ctr)),hp));
  setwlmopts('fa',fareparam(1:2,1:2),'upd',wlmupd(upd));
  if (~numj) % TBD analytic derivatives for plane
    setwlmopts('dfdd','num','dfda','num','d2fdadd','num');
  else setwlmopts('dfdd','num','dfda','num','d2fdadd','num'); end
  setenmo(wlmopts);
  [a,res,S] = wlm(@plane,xyz,E,a,wlmopts{:});
  p = upd(a); p.residual = res; rr = rexp(p.r); zl = rr(:,3);
  sst('(fit plane including input uncertainty)');
  if (~dbgpause(p, 'fit plane residual=%g, c=%s\n%s\n',res,m2s(p.c),r2s(p.r)))
    return;
  end
  
  if (docovar)
    Sq = sum(E,3)./(nd*nd); qavg = mean(xyz)';
    dR = drexp(p.r); dzdr = [dR(:,3,1) dR(:,3,2) dR(:,3,3)];
    Sr = z33; Sr(1:2,1:2) = S(1:2,1:2); Szl = dzdr*Sr*dzdr';
    Src = S; S = zeros(11,11);
    S(1:3,1:3) = Szl; S(4:6,4:6) = Sq; S(7:11,7:11) = Src;
    dcdz = zl*(p.c-qavg)'+zl'*(p.c-qavg)*i33;
    dcdq = i33-zl*zl'; dcdr = dcdz*dzdr(:,1:2); dcdc = zl*zl';
    z23 = zeros(2,3); J=[[z23;dcdz],[z23;dcdq],[eye(2,2);dcdr],[z23;dcdc]];%5x11
    p.covar = J*S*J'; nk = 0; nr = 2;
  end
  
end

% STEP 2,3,4: continue surface fitting if not plane
% also determine boundary type and initialize bounding parameters
switch (st)
  
  case 'a'; dbgmsg('fitting paraboloid with input uncertainty by wlm\n');
    
    sip = 1; % surface type temporarily in play
    
    if (ccon)
      a = [0;0;p.r;0];
      upd = @(a)(rfpp(setp(p,'k',a(1:2),'r',a(3:5),...
                             'c',plane_c+a(6)*plane_n),hp));
    else
      a = [0;0;p.r;p.c];
      upd = @(a)(rfpp(setp(p,'k',a(1:2),'r',a(3:5),'c',a(6:8)),hp));
    end

    setwlmopts('fa',fareparam(3:5,1:3),'upd',wlmupd(upd));
    if (~numj) % analytic derivatives for paraboloid
      setwlmopts('dfdd',@parab_dfdq,'dfda',@parab_dfda)
      setwlmopts('d2fdadd',@parab_d2fdadq);
      setwlmopts('chknd',chknd);
    else setwlmopts('dfdd','num','dfda','num','d2fdadd','num'); end
    setenmo(wlmopts);
    [a,res,S] = wlm(@parab,xyz,E,a,wlmopts{:});
    p = upd(a);
    
    p.residual = res; k = a(1:2);
    dbgmsg('fit paraboloid residual=%g, k=%s, c=%s\n%s\n', ...
           res, m2s(k), m2s(p.c), r2s(p.r));

    if (docovar)
      if (ccon); J = eye(8,6); J(6:8,6) = plane_n; S = J*S*J'; end
      p.covar = S; nk = 2; nr = 3;
    end
      
    % refine into a specific paraboloid type
    
    if (max(abs(k))<ktol), m = 'planar paraboloid';
      
      p.s = 'p'; p.k = [];
      p.r = rcanon2(p.r,'z',3);

      if (docovar)
        drxydr = drcanon2(p.r,'z',2);
        J = zeros(5,8); J(1:2,3:5) = drxydr; J(3:5,6:8) = i33;
        p.covar = J*p.covar*J'; nk = 0; nr = 2;
      end
      
    elseif (min(abs(k))<ktol), m='cylindric paraboloid';
      
      p.s = 'y'; p.b = 'r'; p.d = db(1:2);
      kx = k(1); ky = k(2);
     
      if (docovar); J = [zeros(7,1), eye(7,7)]; end

      if (abs(kx)>abs(ky)), dbgmsg('swapping curvatures\n');
        
        kx = k(2); ky = k(1); ww = [yh,-xh,zh];
        rr = rexp(p.r); rr = rr*ww; p.r = rlog(rr);
        
        if (docovar)
          drdrr = drlog(rr,'m'); drrdr = drexp(p.r,'m')*ww;
          drdr = drdrr*drrdr;
          J(1,1:2) = [1 0]; J(2:4,3:5) = drdr;
        end
        
      end
      
      p.k = ky; nk = 1;
      
      if (docovar); p.covar = J*p.covar*J'; end
      
    elseif (abs(k(1)-k(2))<ktol), m = 'circular paraboloid';
      
      p.s = 'o'; p.b = 'c'; p.d = db(1); p.k = mean(k);
      p.r = rcanon2(p.r,'z',3);

      if (docovar)
        drxydr = drcanon2(p.r,'z',2);
        J = zeros(6,8); J(1,1:2) = [0.5 0.5]; J(2:3,3:5) = drxydr;
        J(4:6,6:8)=i33; p.covar = J*p.covar*J'; nk = 1; nr = 2;
      end
      
    else % elliptic or hyperbolic
      
      if (sign(k(1))==sign(k(2))), m = 'elliptic paraboloid'; p.s = 'e';
      else m = 'hyperbolic paraboloid'; p.s = 'h';
      end
      
      p.k = k; % initially assume abs(kx) is larger
      
      kx = k(1); ky = k(2);
      
      if (abs(kx)>abs(ky)), dbgmsg('swapping curvatures\n'); % swap kx and ky
        
        p.k = [ky, kx];
        ww = [yh,-xh,zh]; rr = rexp(p.r); rr = rr*ww; p.r = rlog(rr);
        
        if (docovar)
          drdrr = drlog(rr,'m');
          J = eye(8,8);
          drrdr = drexp(p.r,'m')*ww;
          drdr = drdrr*drrdr;
          J(1:2,1:2) = [0 1; 1 0]; J(2:4,3:5) = drdr;
          p.covar = J*p.covar*J';
        end
        
      end % swap curvatures
    end % elliptic or hyperbolic
    
    sip = 0; % surface type no longer in play
    
    rfpp(p,hp); sst('(fit paraboloid)');
    if (~dbgpause(p,'detected %s\n',m)), return; end
    
  case 's'; dbgmsg('fitting sphere with input uncertainty by wlm\n');
    
    p.s = 's'; p.b = 'c'; p.d = db(1); p.k = 0; p.r = rcanon2(p.r,'z',3);
    
    a = [0;p.c];
    upd = @(a)(rfpp(setp(p,'k',a(1),'c',a(2:4)),hp));
    setwlmopts('fa',[],'upd',wlmupd(upd));
    if (~numj) % TBD analytic derivatives for sphere
      setwlmopts('dfdd','num','dfda','num','d2fdadd','num');
    else setwlmopts('dfdd','num','dfda','num','d2fdadd','num'); end
    setenmo(wlmopts);
    [a,res,S] = wlm(sphere(zl),xyz,E,a,wlmopts{:});
    p = upd(a); p.residual = res; p.k = a(1);
    sst('(fit sphere)');
    if (~dbgpause(p,'fit sphere residual=%g, radius=%g, c=%s\n%s\n',...
                  res, abs(1/p.k), m2s(p.c), r2s(p.r)))
      return;
    end

    if (docovar)
      Skc = S; S = zeros(6,6);
      S(1:2,1:2) = p.covar(1:2,1:2); S(3:6,3:6) = Skc;
      J = zeros(6,6);
      i22 = eye(2,2); J(2:3,1:2) = i22; J(1,3) = 1; J(4:6,4:6) = i33;
      p.covar = J*S*J'; nk = 1; nr = 2;
    end
      
  case 'c'; dbgmsg('fitting circ cyl with input uncertainty by wlm\n');
    
    p.s = 'c'; p.b = 'r'; p.d = db(1:2); p.k = 0;
    
    a = [0;rcanon2(p.r,'x',2);p.c];
    upd = @(a)(rfpp(setp(p,'k',a(1),'r',rlog(cylrr(a,zl)),'c',a(4:6)),hp));
    setwlmopts('fa',fareparam(2:3,2:3),'upd',wlmupd(upd));
    if (~numj) % TBD analytic derivatives for circ cyl
      setwlmopts('dfdd','num','dfda','num','d2fdadd','num');
    else setwlmopts('dfdd','num','dfda','num','d2fdadd','num'); end
    setenmo(wlmopts);
    [a,res,S] = wlm(ccyl(zl),xyz,E,a,wlmopts{:});
    p = upd(a); p.residual = res; p.k = a(1);
    sst('(fit circular cylinder)');
    if (~dbgpause(p,'fit circ cyl residual=%g, radius=%g, c=%s\n%s\n',...
                  res, abs(1/p.k), m2s(p.c), r2s(p.r)))
      return;
    end

    if (docovar)
      Sr = z33; Sr(2:3,2:3) = S(2:3,2:3);
      dR = drexp(p.r); dxdr = [dR(:,1,1) dR(:,1,2) dR(:,1,3)];
      Sx = dxdr*Sr*dxdr';
      Skc = S([1,4:6],[1,4:6]);
      S = zeros(10,10); S(1:3,1:3) = Szl; S(4:6,4:6) = Sx; S(7:10,7:10) = Skc;
      [rr,xl] = cylrr(a,zl); drdrr = drlog(rr,'m');
      drrdz = [zeros(9,1), reshape(cpm(xl)',9,1), reshape(i33,9,1)];
      drdz = drdrr*drrdz;
      drrdx = [reshape(i33,9,1), reshape(cpm(zl),9,1), zeros(9,1)];
      drdx = drdrr*drrdx;
      J = zeros(7,10);
      J(2:4,1:3) = drdz; J(2:4,4:6) = drdx; J(1,7) = 1; J(5:7,8:10) = i33;
      p.covar = J*S*J'; nk = 1; nr = 3;
    end
    
  case 'p'; % plane, already fit
    
    p.s = 'p'; p.b = bt; p.k = [];
    
    switch (bt) % init boundary parameters p.d
      case 'c'; p.d = db(1);
      case {'e','r'}; p.d = db(1:2);
      case 'q'; p.d = db(1:5);
    end
    
end

if (fb||rbd) % fit boundary or return projected data

  % STEP 5: project to local frame XY plane

  rrinv = rexp(-p.r);
  
  %[ux,uy,uz] = samplecvt('x',sx,'y',sy,'z',sz,'xform',[-p.r,-rrinv*p.c]);

  % save significant time by doing the calcs here
  if ((bsmax > 0) && (bsmax < nd))
    vi = randsample(nd,bsmax); sx = x(vi); sy = y(vi); sz = z(vi);
  else sx = x; sy = y; sz = z;
  end
  uxyz = [sx,sy,sz]*rrinv'+repmat((-rrinv*p.c)',nd,1);
  ux = uxyz(:,1); uy = uxyz(:,2); uz = uxyz(:,3);
  
  if (rbd), p.fitbd = [ux, uy, uz]; end
  
  nu = length(ux); p.fitbn = nu;
  dbgmsg('%d projected data points\n',nu);
  
end

if (fb) % fit boundary
  
  dbgmsg('fitting boundary\n');

  if (docovar)
    Skrc = p.covar;
    drrinvdr = drexp(-p.r); duds = rrinv; dudc = -rrinv;
    Sm = zeros(5,5); dmdr = zeros(5,nr); dmdc = zeros(5,3);
    for i=1:nu
      dmdu = [xh'; yh'; 2*ux(i)*xh'; 2*uy(i)*yh'; ux(i)*yh'+uy(i)*xh']./nu;
      dmds = dmdu*duds; Sm = Sm+dmds*E(:,:,i)*dmds';
      sc = [sx(i);sy(i);sz(i)]-p.c;
      dudr = [drrinvdr(:,:,1)*sc, drrinvdr(:,:,2)*sc, drrinvdr(:,:,3)*sc];
      dudr = dudr(:,1:nr);
      dmdr = dmdr+dmdu*dudr; dmdc = dmdc+dmdu*dudc;
    end
    dmdkrc = [zeros(5,nk),dmdr,dmdc];
    p.covar = [[Sm+dmdkrc*Skrc*dmdkrc';Skrc*dmdkrc'],[dmdkrc*Skrc;Skrc]];
  end
  
  % (normalized) moments
  mx = mean(ux); my = mean(uy);
  vx = mean(ux.*ux); vy = mean(uy.*uy); vxy = mean(ux.*uy);
  % m = [mx my vx vy vxy]'
  
  % STEP 4: (note p.b has already been set)
  if (~strcmpi(p.s,'p')), lambda = sqrt(2)*erfinv(bcp); end
  
  % STEP 6: cylindric parab or circ cyl: aa rect bound
  %
  % **This step also sets p.c as the 1D data centroid along the local frame
  % x axis, which is the symmetry axis of the cylinder.**
  %
  if (any(strcmpi(p.s,{'y','c'})))
    
    p.d = lambda*[sqrt(vx-mx*mx) sqrt(vy)]';
    p.c = rexp(p.r)*mx*xh+p.c;

    if (docovar)
      ddrdm = zeros(2,5);
      ddrdm(1,1) = -lambda*mx/sqrt(vx-mx*mx);
      ddrdm(1,3) = 0.5*lambda/sqrt(vx-mx*mx);
      ddrdm(2,4) = 0.5/sqrt(vy);
      rr = rexp(p.r); drr = drexp(p.r);
      dcdm = [rr(:,1),zeros(3,4)]; dcdr = mx*[drr(:,1,1),drr(:,1,2),drr(:,1,3)];
      J = [zeros(9,5),[zeros(2,7);eye(7,7)]];
      J(1:2,1:5) = ddrdm; J(7:9,1:5) = dcdm; J(7:9,7:9) = dcdr;
      p.covar = J*p.covar*J';
    end
      
  end
  
  % STEP 7: circ parab or sphere: circ bound
  if (any(strcmpi(p.s,{'o','s'})))
    
    p.d = lambda*max(sqrt(vx),sqrt(vy));

    if (docovar)
      ddcdm = zeros(1,5);
      if (vx>vy), ddcdm(1,3) = 0.5*lambda/sqrt(vx);
      elseif (vy<vx), ddcdm(1,4) = 0.5*lambda/sqrt(vy);
      else ddcdm(1,3:4) = 0.5*lambda*[1/sqrt(vx),1/sqrt(vy)];
      end
      J = [[ddcdm;zeros(6,5)],[zeros(1,6);eye(6,6)]];
      p.covar = J*p.covar*J';
    end
    
  end
  
  % STEP 8: ell or hyp parab: ell bound
  if (any(strcmpi(p.s,{'e','h'})))
    
    p.d = lambda*[sqrt(vx) sqrt(vy)]';

    if (docovar)
      ddedm = zeros(2,5);
      ddedm(1,3) = 0.5*lambda/sqrt(vx); ddedm(2,4) = 0.5*lambda/sqrt(vy);
      J = [[ddedm;zeros(8,5)],[zeros(2,8);eye(8,8)]];
      p.covar = J*p.covar*J';
    end

  end
  
  % STEP 9: plane bounds
  if (strcmpi(p.s,'p'))
    
    rr = rexp(p.r);
    p.c = rr*(mx*xh+my*yh)+p.c;
    
    a = vx-mx*mx; b = 2*(vxy-mx*my); c = vy-my*my; % p = [a b c]'
    
    lambda = -log(1-bcp); d = sqrt(b*b+(a-c)*(a-c));
    wp = a+c+d; wn = a+c-d; % w = [wp wn]'
    lp = sqrt(lambda*wp); ln = sqrt(lambda*wn); l = [lp ln]';
    
    if (docovar)
      dldp = 0.5*sqrt(lambda)*...
             [([1 0 1]+[(a-c) b (c-a)]./d)/sqrt(wp); ...
              ([1 0 1]-[(a-c) b (c-a)]./d)/sqrt(wn)];
      dpdm = [-2*mx,   0, 1, 0, 0;
              -my,   -mx, 0, 0, 2;
              0,   -2*my, 0, 1, 0];
      dldm = dldp*dpdm;
      dcdm = rr*[xh, yh, z33];
      drr = drexp(p.r);
      dcdr = [drr(:,:,1)*[mx;my;0], drr(:,:,2)*[mx;my;0]];
      J = eye(10,10);
      J(1:2,1:5) = dldm; J(3:5,1:5) = dpdm;
      J(8:10,1:5) = dcdm; J(8:10,6:7) = dcdr;
      Slprc = J*p.covar*J';
    end
    
    switch (p.b)
      case 'c'; % circ bound
        nd=1; p.d = max(l);
      case {'e'}; % ell bound
        nd=2; p.d = l;
      case {'r'}; % aa rect bound
        nd=2; p.d = l;
      case 'q'; % c quad
        nd=5; d=norm(l); p.d=[d d d d atan2(ln,lp)]';
    end
    
    if (~strcmpi(p.b,'c')) % non-circle bounds
      
      t = 0.5*atan2(b,a-c);
      rr = rexp(p.r); xl = rr(:,1); yl = rr(:,2); zl = rr(:,3);
      xl = cos(t)*xl+sin(t)*yl; yl = cross(zl,xl);
      [p.r, drdrl] = rlog([xl yl zl],'m');

      if (docovar)
        if (strcmpi(p.b,'q')), dddl = [l'./d; l'./d; l'./d; l'./d; [lp -ln]];
        else dddl = eye(2,2); end
        dxldp = [-sin(t); cos(t); 0]*0.5*[-b, (a-c), b];
        dyldp = cpm(zl)*dxldp; dzldp = z33;
        drldp =[reshape(dxldp,3,1,3),reshape(dyldp,3,1,3),reshape(dzldp,3,1,3)];
        drldp = reshape(drldp,9,3);
        dxldr = dxldp*dpdm*dmdr;
        dzldr = [drr(:,3,1), drr(:,3,2)];
        dyldr = [0 0 0; 0 0 1; 0 -1 0]*dzldr+cpm(zl)*dxldr;
        drldr =[reshape(dxldr,3,1,2),reshape(dyldr,3,1,2),reshape(dzldr,3,1,2)];
        drldr = reshape(drldr,9,2);
        drdp = drdrl*drldp; drdr = drdrl*drldr;
        J = [[dddl; zeros(6,2)],...
             [zeros(nd,8); ...
              [drdp,      drdr, z33];
              [z33, zeros(3,2), i33]]];
        p.covar = J*Slprc*J';
      end
      
    else % circle bounds

      if (docovar)
        if (lp > ln), dcdl = [1 0];
        elseif (lp < ln), dcdl = [0 1];
        else dcdl = [0.5 0.5]; % lp == ln
        end
        J = [[[dcdl, zeros(1,3)]; zeros(5,5)],[zeros(1,5); eye(5,5)]];
        p.covar = J*Slprc*J';
      end
      
    end
    
  end % plane bounds
  
  sst('(fit bounds)');
  
else % don't fit boundary
  
  dbgmsg('not fitting boundary\n');
 
  if (docovar)
    nb = 0; % no uncertainty in bound parameters
    switch (p.b)
      case 'c'; nb = 1;
      case 'e'; case 'r'; nb = 2;
      case 'q'; nb = 5;
    end
    S = p.covar; nc = nk+nr+3;
    p.covar = zeros(nb+nc,nb+nc);
    p.covar((nb+1):(nb+nc),(nb+1):(nb+nc)) = S;
  end
  
  sst('(bounds not fit)');
  
end

sec = toc(tstart); p.fitsec = sec;

% run patchresidual
p.ressec = 0;
if (ischar(dores)&&~strcmpi(dores,'alg'))
  [p.residual, p.ressec] = patchresidual(p,'x',x,'y',y,'z',z,'type',dores);
end

% run patchcoverage
p.covsec = 0; p.coverage = [];
if (iscell(docvg)||docvg)
  opts = {}; if iscell(docvg); opts = docvg; end
  [p.coverage, p.covsec] = patchcoverage(p,'x',ux,'y',uy,opts{:});
end

if (dbg) % don't use dbgmsg, this needs to print for any debug level
  fprintf('patchfit: %gs, residual: %gs, coverage: %gs, total: %gs\n',...
          sec, p.ressec, p.covsec, sec+p.ressec+p.covsec);
end

rfpp(p,hp);

dbgpause(p,'patchfit complete: %gs\n',sec+p.ressec+p.covsec);

sst([]); % clear subtitle

end % patchfit
