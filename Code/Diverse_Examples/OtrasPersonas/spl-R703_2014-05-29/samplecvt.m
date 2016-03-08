function varargout = samplecvt(varargin)
% samplecvt(...) converts between xyz and range samples, applies noise, etc
%
%   Data may be suplied in either range or xyz format, and it may also be
%   returned in either format.  The input format is inferred from the given
%   arguments.
%
%   Arguments 'x', 'y', 'z': must either be all present or all absent.
%   Mutually exclusive with the group 'mx', 'my', 'mz', 'r'.  If present,
%   the input is xyz data; the values of each must be arrays of the same
%   size MxN.
%
%   Arguments 'mx', 'my', 'mz', 'r': must either be all present or all
%   absent. Mutually exclusive with the group 'x', 'y', 'z'.  If present
%   the input is range data; the values of each must be arrays of the same
%   size MxN.
%
%   The output format is inferred from the number of outputs requested:
%
%   * three outputs imply xyz format, given as LxK matrices x, y, z in that
%   order
%
%   * four outputs imply range format, given as LxK matrices mx, my, mz, r,
%   in that order
%
%   L and K are implied from M and N and options that may change the data size
%   (clean, unique, decimate, lowres, subsample).
%
%   Range always corresponds to xyz by the invariants
%
%   x = cx+mx.*r
%   y = cy+my.*r
%   z = cz+mz.*r
%
%   If six additional outputs are requested (in either case) they are the
%   covariances cxx, cyy, czz, cxy, cyz, cxz (also all MxN) of the output data
%   in xyz.
%
%   All arguments must be given as name,value pairs. Unrecognized names
%   cause warnings.  If a name is given more than once the last-given
%   (rightmost) value takes precedence.  The order of operations is 
%
%   coerce
%   markrange
%   markdepth
%   markplane
%   marksphere
%   markbad
%   clean
%   unique
%   decimate
%   lowres
%   subsample
%   xform
%   recenter
%   perturb
%   noise
%   project
%   normalize
%   vectorize
%   matricize
%
%   OPTIONS
%
%   Options 'cx', 'cy', 'cz' (default 0): coordinates of measurement ray
%   starting points for range data.  May each either be scalar or MxN.
%
%   Options 'cxx', 'cyy', 'czz', 'cxy', 'cyz', 'cxz' (default 0):
%   covariance of the data.  May each either be scalar or MxN.
%
%   Option 'coerce' (default []): if non-empty then must be either 'single'
%   or 'double'.  All input arrays are coerced to the indicated type.
%
%   Option 'markrange' (default 0): if greater than zero, then mark the data
%   whose range is greater than this value with NaN; if less than zero, then
%   mark the data whose range is less than abs(markrange) with NaN.
%
%   Option 'markdepth' (default 0): if greater than zero, then mark the data
%   whose z coord is greater than this value with NaN; if less than zero, then
%   mark the data whose z coord is less than abs(markrange) with NaN.
%
%   Option 'markplane' (default []): either empty, 1x4 or 4x1.  In the latter
%   cases the first three elements give a 3x1 plane normal vector n and the last
%   the perpendicular distance d from the plane to the origin (measured in units
%   of norm(n)).  Points p "outside" the plane satisfying p'*n > d are marked
%   with NaN.
%
%   Option 'marksphere' (default []): either empty, 1x4 or 4x1.  In the latter
%   cases the first three elements give a 3x1 sphere center c and the last
%   the sphere radius r.  If r > 0 points p outside the sphere satisfying
%   norm(p-c)>r are marked with NaN; if r < 0 points p inside the sphere
%   satisfying norm(p-c)<abs(r) are marked with NaN.
%
%   Option 'markbad' (default []): either empty or a scalar to replace every
%   nan, inf, or complex datum with.
%
%   Option 'clean' (default 0): whether to cull any nan, inf, or complex
%   data. The corresponding ray starting points and covariances, if any, are
%   also removed.  Implies vectorize=1.
%
%   Option 'unique' (default 0): whether to remove duplicate data points. The
%   corresponding ray starting points and covariances, if any, are also removed.
%   Implies vectorize=1;
%
%   Option 'decimate' (default 0): if greater than 1 then the input data
%   matrices will be decimated by the given factor.  E.g. decimate=2 will
%   discard rows and columns 2,4,6,etc.; decimate=3 will discard rows and
%   columns 2,3,5,etc.
%
%   Option 'lowres' (default 0): lowers the resolution. If lowres is greater
%   than 1 then the input data matrices' resolution will be lowered by the given
%   factor by block averaging.  E.g. lowres=2 will replace each 2-by-2 submatrix
%   with its average value using nanmean(). The corresponding ray starting
%   points and covariances, if any, are also averaged.
%
%   Option 'subsample' (default 0): if in (0,1) then all but the requested
%   fraction of the input data is discarded.  If greater than 1 but less than
%   M*N then the data is subsampled to yeild exactly the requested number of
%   subsamples.  The corresponding ray starting points and covariances, if any,
%   are also reoved. Subsampling implies vectorize=1.
%
%   Option 'vectorize' (default 0): whether to make all outputs column vectors.
%
%   Option 'nr' (default 0): if greater than zero the outputs are "matricized"
%   to (nr)x(nv/nr) column-major (first nr points go down first column of
%   returned matrix), where nv is the total number of vertices (nv must be
%   divisible by nr).  Mutually exclusive with 'nc'.
%
%   Option 'nc' (default 0): if greater than zero the outputs are "matricized"
%   to (nv/nc)x(nc) row-major (first nc points go across the first row of
%   returned matrix), where nv is the total number of vertices (nv must be
%   evenly divisible by nc).  Mutually exclusive with 'nr'.
%
%   Option 'normalize' (default 0): if positive, and if output data is requested
%   in range format, gives the length of the returned measurement vectors.
%
%   Option 'xform' (default []): apply given transform to the data. Must be
%   either empty (treated as identity), 3x4, 4x4, or 3x2. If four columns are
%   given, the first three rows are taken as the first three rows of a
%   homogenous rigid body transformation matrix (the rigid body constraints are
%   not checked).  If two columns are given, the second gives a translation
%   vector and the first a rotation vector for rexp(). If covariances are given
%   they are transformed as well.
%
%   Option 'recenter' (default 0): whether to subtract the xyz mean from the
%   data.
%
%   Option 'perturb' (default 0): whether to perturb the data in xyz using
%   Gaussian noise with the specified covariances.
%
%   Option 'noise' (default 0): if positive then this gives the standard
%   deviation of Gaussian white noise added to the data in xyz.
%
%   Option 'project' (default []): if nonempty this must be either 1, 2, or
%   3. The data is projected onto the coordinate plane perpendicular to the
%   indicated 3D axis.
%
%   Option 'dbg' (default 0): may have any nonnegative value. dbg=0 disables
%   debug.  Larger values enable successively more verbose debug.
%
% Copyright (C) 2013 Marsette A. Vona and Dimitrios Kanoulas

tstart = tic();

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

oxyz = ((nargout==3)||(nargout==9));
orng = ((nargout==4)||(nargout==10));
ocov = ((nargout==9)||(nargout==10));

if ((~oxyz)&&(~orng)); error('unsupported number of output arguments'); end

ixyz = 0; % set if input is detected as xyz
irng = 0; % set if input is detected as range

% option defaults
ix = []; iy = []; iz = []; ir = [];
cx = 0; cy = 0; cz = 0;
cxx = 0; cyy = 0; czz = 0; cxy = 0; cyz = 0; cxz = 0;
coerce = [];
markrange = 0; markdepth = 0; markplane = []; marksphere = [];
markbad = []; clean = 0; unq = 0;
decimate = 0; lowres = 0; subsample = 0;
xform = []; recenter = 0;
perturb = 0; noise = 0;
project = [];
normalize = 0; vectorize = 0; nr = 0; nc = 0;
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'x'; ix=v; ixyz=1; case 'y'; iy=v; ixyz=1; case 'z'; iz=v; ixyz=1;
      case 'mx'; ix=v; irng=1; case 'my'; iy=v; irng=1; case 'mz'; iz=v; irng=1;
      case 'r'; ir=v; irng=1;
      case 'cx'; cx = v; case 'cy'; cy = v; case 'cz'; cz = v;
      case 'cxx'; cxx = v; case 'cyy'; cyy = v; case 'czz'; czz = v;
      case 'cxy'; cxy = v; case 'cyz'; cyz = v; case 'cxz'; cxz = v;
      case 'coerce'; coerce = v;
      case 'markrange'; markrange = v; case 'markdepth'; markdepth = v;
      case 'markplane'; markplane = v; case 'marksphere'; marksphere = v;
      case 'markbad'; markbad = v;
      case 'clean'; clean = v; case 'unique'; unq = v;
      case 'decimate'; decimate = v; case 'lowres'; lowres = v;
      case 'subsample'; subsample = v;
      case 'xform'; xform = v; case 'recenter'; recenter = v;
      case 'perturb'; perturb = v; case 'noise'; noise = v;
      case 'project'; project = v;
      case 'normalize'; normalize = v; case 'vectorize'; vectorize = v;
      case 'nr'; nr = v; case 'nc'; nc = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

% check input constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (~xor(ixyz,irng))
  error('must give either x, y, z, or mx, my, mz, r');
end

sz = size(ix); n = sz(1)*sz(2);
if (any(size(iy)~=sz)||(any(size(iz)~=sz))||(irng&&(any(size(ir)~=sz))))
  error('supplied data must all be same size');
end

if (~isscalar(cx)||~isscalar(cy)||~isscalar(cz))
  e = 0;
  if (isscalar(cx)); cx = cx*ones(sz); elseif (any(size(cx)~=sz)); e = 1; end;
  if (isscalar(cy)); cy = cy*ones(sz); elseif (any(size(cy)~=sz)); e = 1; end;
  if (isscalar(cz)); cz = cz*ones(sz); elseif (any(size(cz)~=sz)); e = 1; end;
  if (e); error('supplied data must all be same size'); end
end

if (ocov||(~isscalar(cxx)||~isscalar(cyy)||~isscalar(czz)||...
    ~isscalar(cxy)||~isscalar(cyz)||~isscalar(cxz)))
  e = 0;
  if (isscalar(cxx)); cxx=cxx*ones(sz); elseif (any(size(cxx)~=sz)); e = 1; end;
  if (isscalar(cyy)); cyy=cyy*ones(sz); elseif (any(size(cyy)~=sz)); e = 1; end;
  if (isscalar(czz)); czz=czz*ones(sz); elseif (any(size(czz)~=sz)); e = 1; end;
  if (isscalar(cxy)); cxy=cxy*ones(sz); elseif (any(size(cxy)~=sz)); e = 1; end;
  if (isscalar(cyz)); cyz=cyz*ones(sz); elseif (any(size(cyz)~=sz)); e = 1; end;
  if (isscalar(cxz)); cxz=cxz*ones(sz); elseif (any(size(cxz)~=sz)); e = 1; end;
  if (e); error('supplied data must all be same size %dx%d',sz(1),sz(2)); end
end

if (~isempty(markplane)&&(~isvector(markplane)||length(markplane)~=4))
  error('markplane must be empty, 1x4, or 4x1');
end

if (~isempty(marksphere)&&(~isvector(marksphere)||length(marksphere)~=4))
  error('marksphere must be empty, 1x4, or 4x1');
end

if (~isempty(markbad)&&(~isscalar(markbad)))
  error('markbad must be empty or scalar');
end

if (decimate~=round(decimate)); error('decimate must be an integer'); end
if (lowres~=round(lowres)); error('lowres must be an integer'); end
  
% coerce %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (~isempty(coerce))
  switch (coerce)
    case 'single'; cf = @single;
    case 'double'; cf = @double;
    otherwise; error('unsupported coercion type %s',coerce);
  end
  ix = cf(ix); iy = cf(iy); iz = cf(iz); ir = cf(ir);
  cx = cf(cx); cy = cf(cy); cz = cf(cz);
  cxx = cf(cxx); cyy = cf(cyy); czz = cf(czz);
  cxy = cf(cxy); cyz = cf(cyz); cxz = cf(cxz);
  markrange = cf(markrange); markdepth = cf(markdepth);
  markplane = cf(markplane); marksphere = cf(marksphere); markbad = cf(markbad);
  normalize = cf(normalize); perturb = cf(perturb); noise = cf(noise);
  xform = cf(xform);
end

% range -> xyz %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ((~ixyz)&& ...
    (oxyz||markdepth||~isempty(markplane)||~isempty(marksphere)|| ...
     ~isempty(xform)||recenter||perturb||(noise>0)||~isempty(project)))
  ix = cx+ix.*ir; iy = cy+iy.*ir; iz = cz+iz.*ir;
  ixyz = 1; irng = 0;
end

% mark range, depth, plane, sphere %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (markrange||markdepth||~isempty(markplane)||~isempty(marksphere))
  bi = zeros(sz);
end

if (markrange)
  if (ixyz)
    dx = ix-cx; dy = iy-cy; dz = iz-cz;
    ir = sqrt(dx.*dx+dy.*dy+dz.*dz);
  end
  if (markrange>0); bi=bi|(ir>markrange); else bi=bi|(ir<abs(markrange)); end
end

if (markdepth)
  assert(ixyz~=0);
  if (markdepth>0); bi=bi|(iz>markdepth); else bi=bi|(iz<abs(markdepth)); end
end

if (~isempty(markplane))
  assert(ixyz~=0);
  n = markplane(1:3); n = n(:); d = markplane(4);
  xyz = [ix(:), iy(:), iz(:)];
  bi = bi|reshape((xyz*n)>d,sz);
end

if (~isempty(marksphere))
  assert(ixyz~=0);
  c = marksphere(1:3); r = marksphere(4);
  dx = ix-c(1); dy = iy-c(2); dz = iz-c(3);
  dr = sqrt(dx.*dx+dy.*dy+dz.*dz);
  if (r>0); bi = bi|(dr>r); else bi = bi|(dr<abs(r)); end
end

if (markrange||markdepth||~isempty(markplane)||~isempty(marksphere))
  ix(bi) = nan; iy(bi) = nan; iz(bi) = nan; if (irng); ir(bi) = nan; end
end

% markbad, clean, unique %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (clean||~isempty(markbad))

  vi = (((imag(ix)==0)&(imag(iy)==0)&(imag(iz)==0))&...
        ~(isnan(ix)|isnan(iy)|isnan(iz))&...
        ~(isinf(ix)|isinf(iy)|isinf(iz)));
  
  if (irng); vi = vi&((imag(ir)==0)&~isnan(ir)&~isinf(ir)); end
  
  if (~isscalar(cx))
    vi = vi&(((imag(cx)==0)&(imag(cy)==0)&(imag(cz)==0))&...
             ~(isnan(cx)|isnan(cy)|isnan(cz))&...
             ~(isinf(cx)|isinf(cy)|isinf(cz)));
  end
  
  if (~isscalar(cxx))
    ccc = {cxx,cyy,czz,cxy,cyz,cxz};
    for i=1:6; vi = vi&((imag(ccc{i})==0)&~isnan(ccc{i})&~isinf(ccc{i})); end
  end
  
  if (~all(vi))
    if (clean)
      ix = ix(vi); iy = iy(vi); iz = iz(vi); if (irng); ir = ir(vi); end
      if (~isscalar(cx)); cx = cx(vi); cy = cy(vi); cz = cz(vi); end
      if (~isscalar(cxx))
        cxx = cxx(vi); cyy = cyy(vi); czz = czz(vi);
        cxy = cxy(vi); cyz = cyz(vi); cxz = cxz(vi);
      end
      ns = length(ix); sz = [ns, 1]; n = ns; % we just vectorized the inputs
    elseif(~isempty(markbad))
      bi = ~vi; bad = markbad(1);
      ix(bi) = bad; iy(bi) = bad; iz(bi) = bad; if (irng); ir(bi) = bad; end
    end
  end
end

if (unq)
  id = [ix(:),iy(:),iz(:)];
  if (irng); id = [id, ir]; end
  id = unique(id,'rows');
  ix = id(:,1); iy = id(:,2); iz = id(:,3);
  if (irng); ir = id(:,4); end
  ns = length(ix);
  sz = [ns, 1]; n = ns;
end

% decimate, lowres, subsample %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (decimate > 1)
  r = 1:decimate:sz(1); c = 1:decimate:sz(2);
  ix = ix(r,c); iy = iy(r,c); iz = iz(r,c); if (irng); ir = ir(r,c); end
  if (~isscalar(cx)); cx = cx(r,c); cy = cy(r,c); cz = cz(r,c); end
  if (~isscalar(cxx))
    cxx = cxx(r,c); cyy = cyy(r,c); czz = czz(r,c);
    cxy = cxy(r,c); cyz = cyz(r,c); cxz = cxz(r,c);
  end
  sz = [length(r), length(c)]; n = sz(1)*sz(2);
end

br = []; bc = [];

  function out = nmb(in)
  % nanmean blocks
  out = cellfun(@(c)(nanmean(c(:))), mat2cell(in,br,bc));
  end

if (lowres > 1)

  br = lowres*ones(floor(sz(1)/lowres),1);
  if (mod(sz(1),lowres)); br = [br; mod(sz(1),lowres)]; end
  nbr = length(br);

  bc = lowres*ones(1,floor(sz(2)/lowres));
  if (mod(sz(2),lowres)); bc = [bc, mod(sz(2),lowres)]; end
  nbc = length(bc);

  ix = nmb(ix); iy = nmb(iy); iz = nmb(iz);
  if (irng); ir = nmb(ir); end

  if (~isscalar(cx)); cx = nmb(cx); cy = nmb(cy); cz = nmb(cz); end
  
  % average(x1,...,xN) = A*[x1;...;xN], A = (1/N)[eye(3),...,eye(3)]
  % var(average(x1,...,xN)) = A*E*A', E = blkdiag(E1,...,EN)
  % var(average(x1,...,xN)) = (1/(N*N))(E1+...+EN)
  if (~isscalar(cxx))
    nn = repmat(br,1,nbc).*repmat(bc,nbr,1); nn = nn.*nn;
    cxx = nmb(cxx)./nn; cyy = nmb(cyy)./nn; czz = nmb(czz)./nn;
    cxy = nmb(cxy)./nn; cyz = nmb(cyz)./nn; cxz = nmb(cxz)./nn;
  end
  
  sz = [nbr nbc]; n = nbr*nbc;

end

if ((subsample>0)&&(subsample<n))
  ns = subsample; if (ns < 1); ns = round(ns*n); end
  vi = randsample(n,ns);
  ix = ix(vi); iy = iy(vi); iz = iz(vi); if (irng); ir = ir(vi); end
  if (~isscalar(cx)); cx = cx(vi); cy = cy(vi); cz = cz(vi); end
  if (~isscalar(cxx))
    cxx = cxx(vi); cyy = cyy(vi); czz = czz(vi);
    cxy = cxy(vi); cyz = cyz(vi); cxz = cxz(vi);
  end
  sz = [ns, 1]; n = ns;
end

% form covariance matrices E, either 3x3 or 3x3x(n) %%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ((ocov&&(~isempty(xform)))||perturb)
  nn = n; if (isscalar(cxx)); nn = 1; end
  cxx=reshape(cxx,1,1,nn); cyy=reshape(cyy,1,1,nn); czz=reshape(czz,1,1,nn);
  cxy=reshape(cxy,1,1,nn); cyz=reshape(cyz,1,1,nn); cxz=reshape(cxz,1,1,nn);
  E = [cxx, cxy, cxz; cxy, cyy, cyz; cxz, cyz, czz];
  ncov = nn;
end

% xform, recenter %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (~isempty(xform))
  assert(ixyz~=0);
  [nxr, nxc] = size(xform);
  if ((nxc==4)&&((nxr==3)||(nxr==4))); t = xform(1:3,4); rr = xform(1:3,1:3);
  elseif ((nxc==2)&&(nxr==3)); t = xform(:,2); rr = rexp(xform(:,1));
  else error('xform must be 3x4, 4x4, or 3x2');
  end
  [ix, iy, iz] = xform3(ix, iy, iz, rr, t);
  [cx, cy, cz] = xform3(cx, cy, cz, rr, t);
  if (ocov||perturb); for i=1:ncov; E(:,:,i) = rr*E(:,:,i)*rr'; end; end
  if (ocov)
    cxx = E(1,1,:); cyy = E(2,2,:); czz = E(3,3,:);
    cxy = E(1,2,:); cyz = E(2,3,:); cxz = E(1,3,:);
  end
end

if (recenter)
  mx = nanmean(ix(:)); my = nanmean(iy(:)); mz = nanmean(iz(:));
  ix = ix-mx; iy = iy-my; iz = iz-mz;
  cx = cx-mx; cy = cy-my; cz = cz-mz;
end

% perturb, noise, project %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (perturb)
  assert(ixyz~=0);
  for i=1:n
    p = [ix(i),iy(i),iz(i)]';
    % have to use eig, not chol, to allow for positive semi-definite covar
    if ((ncov>1)||(i==1)); [v,d] = eig(E(:,:,i)); end
    tol = sqrt(sqrt(eps));
    if (~all(diag(d)>=-tol))
      error('covariance must be positive semi-definite');
    end
    d(d<0) = 0;
    p = p+v*sqrt(d)*v'*randn(3,1);
    ix(i) = p(1); iy(i) = p(2); iz(i) = p(3);
  end
end

if (noise>0)
  assert(ixyz~=0);
  nn = noise*eye(3,3);
  for i=1:n
    p = [ix(i),iy(i),iz(i)]';
    p = p+nn*randn(3,1);
    ix(i) = p(1); iy(i) = p(2); iz(i) = p(3);
  end
end

if (~isempty(project))
  assert(ixyz~=0);
  if (~isscalar(project)||(project<1)||(project>3))
    error('project must be empty or a scalar in [1,3]');
  end
  switch (project)
    case 1; ix = zeros(sz); case 2; iy = zeros(sz); case 3; iz = zeros(sz);
  end
  if (ocov)
    switch (project)
      case 1; cxx = 0*cxx; cxy = 0*cxy; cxz = 0*cxz;
      case 2; cyy = 0*cyy; cxy = 0*cxy; cyz = 0*cyz;
      case 3; czz = 0*czz; cxz = 0*cxz; cyz = 0*cyz;
    end
  end
end

% generate outputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (oxyz)
  assert(ixyz~=0);
  varargout{1} = ix; varargout{2} = iy; varargout{3} = iz;
else
  assert(orng~=0);
  % xyz -> range, normalize
  nn = 1; if (normalize > 0) nn = normalize; end
  if (ixyz)
    dx = ix-cx; dy = iy-cy; dz = iz-cz;
    ir = sqrt(dx.*dx+dy.*dy+dz.*dz);
    ix = ix.*(nn./ir); iy = iy.*(nn./ir); iz = iz.*(nn./ir);
    ir = ir/nn;
    ixyz = 0; irng = 1;
  elseif (nn > 0)
    ml = sqrt(ix.*ix+iy.*iy+iz.*iz);
    mx = ix.*(nn./ml); my = iy.*(nn./ml); mz = iz.*(nn./ml);
  end
  assert(irng~=0);
  varargout{1} = ix; varargout{2} = iy; varargout{3} = iz; varargout{4} = ir;
end

if (ocov)
  cxx = reshape(cxx,sz); cyy = reshape(cyy,sz); czz = reshape(czz,sz);
  cxy = reshape(cxy,sz); cyz = reshape(cyz,sz); cxz = reshape(cxz,sz);
  i = 4; if (orng); i = 5; end
  varargout{i+0} = cxx; varargout{i+1} = cyy; varargout{i+2} = czz;
  varargout{i+3} = cxy; varargout{i+4} = cyz; varargout{i+5} = cxz;
end

% reshape outputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (vectorize)
  for i=1:length(varargout); varargout{i} = varargout{i}(:); end
end

if (nr>0)
  if (nc>0); error('nr and nc are mutually exclusive'); end
  if ((nr~=round(nr))||mod(n,nr))
    error('n must be evenly divisible by nr');
  end
  for i=1:length(varargout); varargout{i} = reshape(varargout{i},nr,[]); end
elseif (nc>0)
  if ((nc~=round(nc))||mod(n,nc))
    error('n must be evenly divisible by nc');
  end
  for i=1:length(varargout); varargout{i} = reshape(varargout{i},nc,[])'; end
end

if (dbg); fprintf('samplecvt: %gs\n',toc(tstart)); end

end
