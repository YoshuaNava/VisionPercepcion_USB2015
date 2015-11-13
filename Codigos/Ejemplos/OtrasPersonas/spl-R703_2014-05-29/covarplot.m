function h = covarplot(varargin)
% covarplot draws error ellipsoid representations of 3D covariance matrices
%
%   Vectorized; plots zero or more passed covariance matrices.
%
%   Allows positive semi-definite covariance matrices, so one or more axes
%   may have zero radius, representing zero uncertainty in that direction.
%   This allows the same code to also plot 2D error ellipses and 1D
%   confidence intervals.
%
%   If the data is supplied in factored (E,V) form, then covariance
%   matrices with one or more infinite eigenvalues are also supported.
%   Arrows are drawn in the directions corresponding to zero certainty.
%
%   The plot is made in the current axes of the current figure, if any,
%   else a new figure window is opened.  The return value h is an MxN array
%   of handles of Matlab hgtransform groups, each containing all generated
%   graphics for one data point. (M and N are inferred from the inputs, see
%   below).
%
%   All arguments are given as name,value pairs.  Unrecognized names cause
%   warnings.  If a name is given more than once the last-given (rightmost)
%   value takes precedence.
%
%   ARGUMENTS AND OPTIONS
%
%   Either c**, E, or E and V are required; all others are optional.
%
%   Option 'cxx', 'cyy', 'czz', 'cxy', 'cyz', 'cxz' (default 0): data
%   variances and covariances.  May each either be present or absent and
%   scalar or MxN. Mutually exclusive with 'E' and 'V'.
%
%   Option 'E' (default []): either [], Dx1xM, or DxDxM.  Ignored if empty,
%   otherwise mutually exclusive with 'cxx'--'cxz'. If DxDxM then E(:,:,i)
%   are the data covariance matrices and 'V' cannot also be specified. If
%   Dx1xM then V(:,:,i)*diag(E(:,1,i))*V(:,:,i)' are the data covariance
%   matrices and V must be DxDxM.  D must be in [1,3]; if D<3 then the
%   given covariance matrices are extended to 3x3 by appending zeros to the
%   right and bottom. In all these cases other per-datum inputs (e.g. cx, cy,
%   cz) are expected to be Mx1 and the output data will be Mx1.
%
%   Option 'V' (default []): either [] or 3x3xM. Ignored if empty,
%   otherwise see 'E'.
%
%   Options 'cx', 'cy', 'cz' (default 0): ellipsoid centers. May each
%   either be present or absent and scalar or MxN.
%
%   Option 'p' (default 0.95): confidence interval in [0,1), scales radii
%   so that the data is expected to lie within the ellipsoid with the given
%   probability. Scalar or MxN.
%
%   Option 'ns' (default 30): number of segments used to draw ellipses.
%   Scalar.
%
%   Option 'lw' (default 0.5): line width in points. Scalar.
%
%   Option 'lc' (default 'k'): line color, either a scalar color name or
%   [r g b].
%
%   Option 'ts' (default '0.05'): tail scale for error bars and arrows.
%
%   Options 'ag', 'ar': see axescfg()
%
%   Option 'parent' (default []): if nonempty then this is the parent for
%   all generated hgtransforms.
%
%   Option 'dp' (default 0): whether to draw the center points.
%
%   Option 'ps' (default 6): size of center point markers in units.
%
%   Option 'pc' (default 'b'): color for center points. Either a colorspec
%   or [r g b].
%
%   Option 'pt' (default 'o'): center point marker type.
%
%   Option 'da' (default 1): whether to draw RGB axes triad for each
%   covariance matrix.
%
%   Option 'as' (default 0): scale factor for local frame axes triad and
%   infinite uncertainty arrows, or non-positive to autocompute as the average
%   of the nonzero finite standard deviations, if any, else 1.
%
%   Option 'db' (default 1): whether to draw an error bars if there is only one
%   finite nonzero standard deviation.
%
%   Option 'de' (default 1): either 0, 1, or 2; whether to draw error ellipses
%   in planes with two finite nonzero standard deviations.  0=never; 1=always;
%   2=only when there are exactly two nonzero finite standard deviations.
%
%   Option 'di' (default 1): whether to draw arrows representing directions of
%   infinite uncertainty.
%
%   Option 'do' (default 1):  whether to draw the ellipsoid surface.
%
%   Option 'fc' (default 'c'): ellipsoid face color, either a color name or [r g
%   b].
%
%   Option 'fa' (default 0.5): ellipsoid face alpha in [0,1].
%
%   Option 'tol' (default sqrt(eps))): stddev tolerance; scalar stdev (square
%   roots of the eigenvalues of the covariance matrix) below this tolerance are
%   considered zero.
%
%   Options 'decimate', 'subsample': see samplecvt().
%
%   'dbg' (optional, default 0): may have any nonnegative value. dbg=0 disables
%   debug. Larger values enable successively more verbose debug.
%
% Copyright (C) 2013 Marsette A. Vona

tstart = tic();

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
cxx = []; cyy = []; czz = []; cxy = []; cyz = []; cxz = [];
E = []; V = [];
cx = 0; cy = 0; cz = 0;
p = 0.95;
ns = 30; lw = 0.5; lc = 'k'; ts = 0.05;
ag = 0; ar = 1;
parent = [];
dp = 0; ps = 6; pc = 'b'; pt = 'o';
da = 1; as = 0;
db = 1; de = 1; di = 1; do = 1;
fc = 'c'; fa = 0.5;
tol = sqrt(eps);
decimate = 0; subsample = 0;
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'cxx'; cxx = v; case 'cyy'; cyy = v; case 'czz'; czz = v;
      case 'cxy'; cxy = v; case 'cyz'; cyz = v; case 'cxz'; cxz = v;
      case 'E'; E = v; case 'V'; V = v;
      case 'cx'; cx = v; case 'cy'; cy = v; case 'cz'; cz = v;
      case 'p'; p = v;
      case 'ns'; ns = v; case 'lw'; lw = v; case 'lc'; lc = v;
      case 'ts'; ts = v;
      case 'parent'; parent = v;
      case 'ag'; ag = v; case 'ar'; ar = v;
      case 'dp'; dp = v; case 'ps'; ps = v;
      case 'pc'; pc = v; case 'pt'; pt = v;
      case 'da'; da = v; case 'as'; as = v;
      case 'db'; db = v; case 'de'; de = v; case 'di'; di = v;
      case 'do'; do = v; case 'fc'; fc = v; case 'fa'; fa = v;
      case 'tol'; tol = v;
      case 'subsample'; subsample = v; case 'decimate'; decimate = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

% ck inputs & regularize so 3x3 covars are vv(:,:,i)*diag(ee(:,1,i))*vv(:,:,i)'

cc = {cxx, cyy, czz, cxy, cyz, cxz};
[ccm, ccn] = cellfun(@size,cc);
if (any(ccm)||any(ccn)) % got c** data
  
  if ((~isempty(E))||(~isempty(V)))
    error('cannot specify both c** and E, V');
  end
  
  m = max(ccm); n = max(ccn);  sz = [m n]; nd = m*n;
  
  ccc = cell(1,6);
  for i=1:6
    if ((ccm(i)==0)||(ccn(i)==0)); ccc{i} = zeros(m,n);
    elseif ((ccm(i)==1)&&(ccn(i)==1)); ccc{i} = cc{i}*ones(m,n);
    elseif ((ccm(i)~=m)||(ccn(i)~=n)); error('nonscalar data must be eql size');
    else ccc{i} = cc{i};
    end
  end
  
  % apply decimation
  if (decimate > 1)
    if (decimate~=round(decimate)) error('decimate must be integer'); end
    r = 1:decimate:m; c = 1:decimate:n;
    for i=1:6; ccc{i} = ccc{i}(r,c); end
    if (~isscalar(cx)); cx = cx(r,c); end
    if (~isscalar(cy)); cy = cy(r,c); end
    if (~isscalar(cz)); cz = cz(r,c); end
    if (~isscalar(p)); p = p(r,c); end
    m = length(r); n = length(c); sz = [m n]; nd = m*n;
  end
  
  cxx=reshape(ccc{1},1,1,nd);
  cyy=reshape(ccc{2},1,1,nd);
  czz=reshape(ccc{3},1,1,nd);
  cxy=reshape(ccc{4},1,1,nd);
  cyz=reshape(ccc{5},1,1,nd);
  cxz=reshape(ccc{6},1,1,nd);
  E = [cxx, cxy, cxz; cxy, cyy, cyz; cxz, cyz, czz];
  
  [er, ec, ep] = size(E);
  
else % should have gotten E, V data
  
  if (isempty(E)); error('must specify either E or c**'); end
  [er, ec, ep] = size(E);
  m = ep; n = 1; sz = [m n]; nd = m;
  
  % apply decimation
  if (decimate > 1)
    if (decimate~=round(decimate)); error('decimate must be integer'); end
    vi = 1:decimate:m;
    E = E(:,:,vi); if (~isempty(V)); V = V(:,:,vi); end
    if (~isscalar(cx)); cx = cx(vi); end
    if (~isscalar(cy)); cy = cy(vi); end
    if (~isscalar(cz)); cz = cz(vi); end
    if (~isscalar(p)); p = p(vi); end
    m = length(vi); sz = [m n]; nd = m*n;
  end
  
end

% at this point data is in E, V form

% now that we know datasize, check passed center data
dd = {cx, cy, cz, p};
for d=dd
  if ((~isscalar(d))&&(size(d)~=sz))
    error('nonscalar data must be same size');
  end
end

% apply subsampling
if ((subsample>0)&&(subsample<n))
  ns = subsample; if (ns < 1); ns = round(ns*nd); end
  vi = randsample(nd,ns);
  E = E(:,:,vi); if (~isempty(V)); V = V(:,:,vi); end
  if (~isscalar(cx)); cx = cx(vi); end
  if (~isscalar(cy)); cy = cy(vi); end
  if (~isscalar(cz)); cz = cz(vi); end
  if (~isscalar(p)); p = p(vi); end
  m = ns; n = 1; sz = [m, 1]; nd = ns;
end

% check for the possible E, V forms and convert to ee, vv

if ((er<1)||(er>3)||((ec~=er)&&(ec~=1)))
  error('E must be DxDxM or Dx1xM with 1<=D<=3');
end

ee = zeros(3,1,nd); % eigenvalues of covariance matrices
vv = zeros(3,3,nd); % eigenvectors of covariance matrices

if (er==ec) % must eigendecompose
  if (~isempty(V)); error('cannot specify V with E DxDxM'); end
  e = zeros(3,3);
  for i=1:nd
    e(1:er,1:ec) = E(1:er,1:ec,i);
    [v, d] = eig(e);
    vv(:,:,i) = v; ee(:,1,i) = diag(d);
  end
else % given factored
  for i=1:nd; vv(:,:,i) = eye(3,3); ee(1:er,1,i) = E(1:er,1,i); end
  if (~isempty(V))
    [vr, vc, vp] = size(V);
    if ((vr~=er)||(vc~=er)||(vp~=nd)); error('V must be DxDxM'); end
    for i=1:nd; vv(1:vr,1:vc,i) = V(1:vr,1:vc,i); end
  end
end

% set variances near zero to zero
ee(abs(ee)<(tol*tol)) = 0;

for i=1:nd
  d = ee(:,1,i);
  if (any(d<0))
    v = vv(:,:,i);
    d, v, v*diag(d)*v'
    error('covar must be positive semi-definite');
  end
end

% get axes
a = newplot(); % gets current or makes new, as appropriate
washold = ishold(); hold('on');

% now do the drawing
h = zeros(nd,1);

% transformation matrix
xm = eye(4,4);

for i = 1:nd
  
  d = sqrt(ee(:,1,i)); % stdev for this point
  
  % indices of positive, finite stdev
  vi = ((~isinf(d))&(d>0)); nvi = sum(vi);
  
  % default axes scale to avg nonzero finite stdev
  if (as<=0)
    if (nvi>0); as = mean(d(vi)); else as = 1; end
  end
  
  % center and scale
  if (isscalar(cx)); x = cx; else x = cx(i); end
  if (isscalar(cy)); y = cy; else y = cy(i); end
  if (isscalar(cz)); z = cz; else z = cz(i); end
  if (isscalar(p)); s = scale(p,d); else s = scale(p(i),d); end
  
  xm(1:3,1:3) = vv(:,:,i);
  xm(1:3,4) = [x; y; z];
  
  h(i) = hgtransform();
  set(h(i),'Matrix',xm);
  if (~isempty(parent)); set(h(i),'Parent',parent); end
  
  % plot ellipsoid
  if (do&&(nvi==3))
    [ex, ey, ez] = ellipsoid(0,0,0,s*d(1),s*d(2),s*d(3),ns);
    surf(ex, ey, ez,'Parent',h(i),'FaceColor',fc,'FaceAlpha',fa,...
         'EdgeColor','none');
  end
  
  % plot axes
  if (da)
    line([0;as],[0;0],[0;0],'Parent',h(i),'Color','r','LineWidth',lw);
    line([0;0],[0;as],[0;0],'Parent',h(i),'Color','g','LineWidth',lw);
    line([0;0],[0;0],[0;as],'Parent',h(i),'Color','b','LineWidth',lw);
  end
  
  % plot up to 3 ellipses
  if ((de==1)||((de==2)&&(nvi==2)))
    kk = [1 3 2];
    ll = [2 1 3];
    mm = [3 2 1];
    for j=1:3
      k = kk(j); l = ll(j); m = mm(j);
      if (((d(k)>0)&&~isinf(d(k)))&&((d(l)>0)&&~isinf(d(l))))
        ellipse(h(i),k,l,m,s*d(k),s*d(l),ns,lw,lc);
      end
    end
  end
  
  % plot up to 3 arrows
  if (di)
    for j=1:3; if (isinf(d(j))); arrow(h(i),j,as*s,ts,lw,lc); end; end
  end
  
  % plot up to one error bar
  if (db&&(nvi==1)); ii = find(vi); ebar(h(i),ii,s*d(ii),ts,lw,lc); end
  
  % plot center point
  if (dp)
    line(0,0,0,'Parent',h(i),...
         'Marker',pt,'MarkerFaceColor',pc,'MarkerEdgeColor',pc,'MarkerSize',ps);
  end
  
end

% make output format same as input
h = reshape(h,sz(1),sz(2));

% if the axes was newly created, or if not but we replaced its old contents,
% then configure it appropriately
if (~washold); hold('off'); end
if (~ishold(a)); axescfg(a,ag,ar); end

if (dbg); fprintf('covarplot: %gs\n',toc(tstart)); end

end % covarplot

function s = scale(p,d)
% compute ellipsoid scale using chi2inv
nd = sum((d>0)&(~isinf(d)));
s = 1; if (nd > 0); s = sqrt(chi2inv(p,nd)); end
end

function ellipse(p,i,j,k,ri,rj,ns,lw,lc)
% draws ellipse in ij plane with normal in direction k
xyz = {[], [], []}; dt = 2*pi/ns;
xyz{i} = ri*[1, cos((1:ns)*dt)]'; xyz{j} = rj*[0, sin((1:ns)*dt)]';
xyz{k} = zeros(ns+1,1);
line(xyz{1},xyz{2},xyz{3},'Parent',p,'LineWidth',lw,'Color',lc);
end

function arrow(p,i,r,ts,lw,lc)
% draws symmetric arrow in axis i with radius r
m = [0 0 0]; m(i) = r;
j = mod(i,3)+1;
drd = [0 0 0]; drd(i) = -ts*r; drd(j) = -ts*r;
dru = [0 0 0]; dru(i) = -ts*r; dru(j) =  ts*r;
dld = [0 0 0]; dld(i) =  ts*r; dld(j) = -ts*r;
dlu = [0 0 0]; dlu(i) =  ts*r; dlu(j) =  ts*r;
lines(p,lw,lc, -m,m, m+drd,m, m,m+dru, -m+dlu,-m, -m,-m+dld);
end

function ebar(p,i,r,ts,lw,lc)
% draws symmetric error bar in axis i with radius r
m = [0 0 0]; m(i) = r;
j = mod(i,3)+1;
d = [0 0 0]; d(j) = ts*r;
lines(p,lw,lc, -m,m, m-d,m+d, -m-d,-m+d);
end

function lines(p,lw,lc,varargin)
% draws 3d lines given as s,e endpoint pairs in varargin
nl = length(varargin)/2;
xx = zeros(2,nl); yy = zeros(2,nl); zz = zeros(2,nl);
for i=1:nl
  s = varargin{2*i-1}; e = varargin{2*i};
  xx(1,i) = s(1); yy(1,i) = s(2); zz(1,i) = s(3);
  xx(2,i) = e(1); yy(2,i) = e(2); zz(2,i) = e(3);
end
line(xx,yy,zz,'Parent',p,'LineWidth',lw,'Color',lc);
end
