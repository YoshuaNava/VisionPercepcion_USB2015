function o = patchchk(p, varargin)
%o = patchchk(p) checks and possibly elaborates patch fields
%
%   The input argument p is the patch to check, a matlab struct with fields
%
%       p.s - surface type
%       p.b - boundary type
%       p.d - boundary parameters
%       p.k - curvature parameters
%       p.r - rexp(p.r) is basis for local frame of patch
%       p.c - origin of local frame of patch
%
%   All fields are optional.  Interpretation of field values and defaults for
%   missing fields is detailed under PRIMARY FIELDS below.
%
%   The output o is a copy of p, possibly elaborated with missing fields set to
%   defaults and/or derived fields calculated. The available derived fields are
%   detailed under DERIVED FIELDS below.
%
%   Name,value pairs may also optionally be given to set options, as
%   detailed below in OPTIONS.
%
%   PRIMARY FIELDS
%
%   The field p.s gives the patch surface type and must be one of the
%   following:
%
%       'e' - elliptic paraboloid
%       'h' - hyperbolic paraboloid
%       'y' - cylindric paraboloid
%       'o' - circular paraboloid
%       'p' - plane
%       's' - sphere
%       'c' - circular cylinder
%
%   If p.s is not present it is inferred as either 'e', 'h', 'y', 'o', or
%   'p' from the curvatures (using defaults for those if necessary).
%
%   The field p.b gives the patch boundary type and must be one of the
%   following:
%
%       'c' - circle
%       'e' - ellipse
%       'r' - axis aligned rectangle
%       'q' - convex quadrilateral
%
%   If p.b is not present it is inferred from surface type and, for planes, from
%   the length of the given boundary parameters p.d.  For planes p.b is always
%   inferred as either 'o', 'e', or 'q' (never 'r');
%
%   Only the following combinations of p.s and p.b are allowed:
%
%       ('e', 'e')
%       ('h', 'e')
%       ('y', 'r')
%       ('o', 'c')
%       ('p', 'c'), ('p', 'e'), ('p', 'r'), ('p', 'q')
%       ('s', 'c')
%       ('c', 'r')
%
%   The fields p.c and p.r, if present and non-empty, must be numeric
%   vectors. As a pair, (p.r, p,c) define the extrinsic rigid-body pose of
%   the patch. p.c is the origin and rexp(p.r) is a basis for the local
%   Cartesian frame in which the patch is defined.
%
%   If either p.c or p.r has length greater than three, the latter elements
%   are ignored.  If either has length less than three, or does not exist,
%   the missing elements are taken as zero.
%
%   The field p.k is a vector of nk curvature constants, where nk depends
%   on the surface type:
%
%                 'e', 'h' : nk=2
%       'y', 'o', 's', 'c' : nk=1
%                      'p' : nk=0
%
%   If p.k has more than nk entries, the latter ones are ignored. If it has
%   fewer entries, or does not exist, the missing curvatures are taken as
%   zero.
%
%   The curvatures for elliptic and hyperbolic paraboloids are given in the
%   order p.k = [kx ky]. Additionally, curvatures are expected to be given
%   in a canonical order, as follows: for elliptic paraboloids nonzero
%   curvatures must have the same sign and the curvatures must satisfy
%   abs(kx)<=abs(ky), and for hyperbolic paraboloids nonzero curvatures
%   must have opposite signs and the curvatures must satisfy
%   abs(kx)<=abs(ky).
%
%   The field p.d is a vector of nd boundary parameters, where nd
%   depends on the boundary type:
%
%            'c' : nd=1
%       'e', 'r' : nd=2
%            'q' : nd=5
%
%   The circle, ellipse, AA rect, and the first four convex quad boundary
%   parameters are radii.  Ellipse and AA rect radii are given in the order p.d
%   = [dx dy]. Convex quad radii are given, along with the half angle gamma in
%   radians between the last and first half-diagonal, in the order p.d = [r1 r2
%   r3 r4 gamma], where rQ is the radius to the quad vertex in quadrant Q.
%
%   All boundary parameters must be positive. If p.d has more than nd
%   entries, the latter ones are ignored. If it has fewer entries, or does
%   not exist, missing radii are taken equal to the last given radius, if
%   any. If no radii are given they are all taken as 1. If gamma is not
%   given it defaults to pi/4.
%
%   DERIVED FIELDS
%
%   The derived fields p.kx and p.ky are the curvatures of the patch in
%   local frame.
%
%   The derived fields p.nk and p.nd are the number of curvature and
%   boundary parameters for the patch surface and boundary types,
%   respectively.
%
%   The derived fields p.rm, p.pm are 3x3 and 4x4 matrices giving a basis
%   for, and the full pose of, the local frame of the patch relative to an
%   extrinsic frame, respectively.  p.rm is a rotation matrix from local to
%   extrinsic frame, and p.pm is a rigid-body homogenous transformation
%   matrix with top three rows [p.rm, p.c] and bottom row [0 0 0 1].
%
%   The derived fields p.sl, p.sg, p.sn, p.zl, p.ri, p.bi, p.bs, p.bl, p.bc are
%   *vectorized* functions:
%
%   sl(x,y,z) is an implicit form of the surface in local frame.
%   sl(x,y,z)==0 iff (x,y,z) is a point on the surface in local frame.
%   Otherwise the return is the algebraic error of the test point: positive
%   for points below the surface and negative for points above the surface.
%
%   sg(xyz) is the gradient of sl(x,y,z).  Input must be Nx3, output is Nx3.
%
%   sn(xyz) is the outward-pointing unit vector surface normal of the
%   surface. Input must be Nx3, output is Nx3.
%
%   zl(x,y) is an explicit form of the surface in local frame, i.e.
%   z=zl(x,y) where (x,y,z) is a point on the surface in local frame.
%
%   ri(mx,my,mz,cx,cy,cz) returns the range(s) of intersection(s) of 3D
%   rays from start point(s) (cx,cy,cz) in the direction(s) (mx,my,mz) with
%   the patch surface. Clipping to the patch boundary is not applied.
%   Failed intersections yeild either infinite or complex range.
%
%   bi(i,u) returns the intersections of an axis-alinged line with the
%   bounding curve in the local frame XY plane: i=1 for vertical line at
%   x=u, i=2 for horizontal line at y=u. Output [a b] is the min=a and
%   max=b intersection, or [nan nan] if none.
%
%   bs(ss,kx,ky) returns boundary curve samples given sample spacing s and
%   curvatures kx and ky.  Output is NBx2 in CCW order.
%
%   bl(x,y) is an implicit form of the boundary curve in the local frame XY
%   plane.  bl(x,y)==0 iff (x,y) is a point on the boundary curve.
%   Otherwise the return is the algebraic error of the test point: positive
%   for points outside the curve, negative for points inside the curve.
%
%   bc(x,y,w) computes the intersection area of the boundary and a square with
%   lower left corner (x*w,y*w) and side length w (which may be either scalar or
%   vector).  Note: in the current implementation x and y must be integers
%   and the computed intersection may be approximate.
%
%   Except where indicated, these functions use the current values of patch
%   parameters and must be regenerated if the parameters are later changed.
%
%   The field p.bb = [xmin, xmax; ymin, ymax] gives the axis aligned bounding
%   box of the boundary curve in the local frame XY plane.
%
%   The field p.ba is the area inside the bounding curve in the local frame
%   XY plane.
%
%   The field p.gv is a 1x3 cell array of NGx1 cell arrays gx, gy, gz. Each
%   entry in the latter is a column vector of sample point coordinates in
%   order along one of the NG gridlines. Note that each gridline may have a
%   different number of vertices.
%
%   The derived fields p.bv, p.fv, and p.ft are (NB+1)x3, NVx3, and NFx3
%   matrices, respectively.  The rows of p.bv and p.fv are the [x, y, z]
%   coordinates of sample points of the boundary curve and face vertices.
%   p.ft gives the indices into p.fv for each face triangle. The last row
%   of p.bv is a copy of the first to make a closed curve.
%
%   The derived field p.ta is the sum of the area of the triangles p.ft.
%
%   All non-boundary samples are taken on an axis-aligned grid at spacing p.ss.
%   For the boundary curve, the samples are taken at the intersections of this
%   grid with the boundary curve, plus at sharp curve corners, if any.  Face
%   vertices are the union of the boundary and non-boundar samples. Grid samples
%   are points on a grid with spacing p.gd*p.ss that are inside or on the
%   boundary curve, where p.gd is the grid decimation factor (default 1, see
%   option 'gd' below).
%
%   OPTIONS
%
%   A list of name,value pairs may be given to set options. Unrecognized
%   names cause warnings. If a name is given more than once the last-given
%   (rightmost) value takes precedence.
%
%   Option 'strict' (default 2): may have the value 0, 1, 2, or 3. strict=0
%   disables input checking; incorrect output or crashes may occur for
%   invalid inputs. strict=1 causes an error for any invalid input.
%   strict=2 causes an error for irrecoverable problems in the input, but
%   issues a warning for input problems that can be tolerated. strict=3
%   only issues warnings.
%
%   Option 'elaborate' (default 'none'): may have the value 'none',
%   'defaults', 'derived', or 'all'. 'none' elaborates nothing. 'defaults'
%   adds default values for any missing primary fields only. 'derived'
%   adds all derived quantities. 'all' adds both defaults and derived
%   quantities. These settings hold unless specifically overridden later in
%   the argument list; for example, the argument list 'elaborate','all',
%   'bv',0 will add defaults and all derived quantities except boundary
%   samples.
%
%   Option 'ke' (default 0): do not change any field which already is set
%   on p and which is non-empty (keep existing).
%
%   Option 'cl' (default 0): removes any extraneous fields, e.g. if
%   length(p.k)>nk or length(p.d)>nd, or if p had extraneous fields.  Implies
%   ke=0.
%
%   Option 'gp' (default 0): whether to generate p.rm, p.pm
%
%   Option 'gk' (default 0): whether to generate p.kx, p.ky
%
%   Option 'gn' (default 0): whether to generate p.nd, p.nk
%
%   Option 'gb' (default 0): whether to generate p.bb, p.ba, p.bi, p.bs, p.bl
%
%   Option 'gf' (default 0): whether to generate p.sl, p.sg, p.sn, p.zl, p.ri
%
%   Option 'gs' (default 0): whether to generate p.gv, p.bv. Implies gb=1 and
%   gf=1.
%
%   Option 'gt' (default 0): whether to generate p.fv, p.ft.  Implies gs=1.
%
%   Option 'ga' (default 0): whether to generate p.ta.  Implies gt=1.
%
%   Option 'ss' (default p.ss if set, else 1): new sample spacing in the local
%   frame XY plane
%
%   Option 'gd' (default p.gd if set, else 1): grid decimation factor relative
%   to ss.  Must be a positive integer.  1 gives no decimation.
%
%   Option 'dbg' (default 0): may have any nonnegative value. dbg=0
%   disables debug. Larger values enable successively more verbose debug.
%
% Copyright (C) 2013 Marsette A. Vona

tstart = tic();

% process args %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (~isscalar(p) || ~isstruct(p)); error('p must be scalar struct'); end

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
strict = 2; elaborate = 'none'; ke = 0; cl = 0;
gp = 0; gk = 0; gn = 0; gb = 0; gf = 0; gs = 0; gt = 0; ga = 0;
ss = 1; if (isfield(p,'ss')&&~isempty(p.ss)); ss = p.ss; end;
gd = 1; if (isfield(p,'gd')&&~isempty(p.gd)); gd = p.gd; end;
dbg = 0;

setdef = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'strict'; strict = v; case 'ke'; ke = v; case 'cl'; cl = v;
      case 'elaborate';
        elaborate = v;
        switch (elaborate)
          case {'none', 'defaults'};
            setdef = strcmpi(elaborate,'defaults');
            gp = 0; gk = 0; gn = 0; gb = 0; gf = 0; gs = 0; gt = 0; ga = 0;
          case {'derived','all'};
            setdef = strcmpi(elaborate,'all');
            gp = 1; gk = 1; gn = 1; gb = 1; gf = 1; gs = 1; gt = 1; ga = 1;
          otherwise; warning('unexpected elaborate %s',elaborate);
        end
      case 'gp'; gp = v; case 'gk'; gk = v; case 'gn'; gn = v;
      case 'gb'; gb = v; case 'gf'; gf = v;
      case 'gs'; gs = v; case 'gt'; gt = v; case 'ga'; ga = v;
      case 'ss'; ss = v; case 'gd'; gd = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

if (cl); ke = 0; end
if (ga); gt = 1; end
if (gt); gs = 1; end
if (gs); gb = 1; gf = 1; end

% check input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% setup input checking
if ((strict < 0) || (strict > 3))
  warning('unexpected strict %s',strict);
  strict = 2;
end

switch (strict)
  case 0; cki = 0; iee = @(m)(fprintf('%s\n',m)); iew = @(m)(fprintf('%s\n',m));
  case 1; cki = 1; iee = @error; iew = @error;
  case 2; cki = 1; iee = @error; iew = @warning;
  case 3; cki = 1; iee = @warning; iew = @warning;
end

% check passed sample spacing and decimation factor
% TBD implement some kind of nice autoset for ss
if (cki && (ss<=0))
  iee('invalid sample spacing');
  ss = 1;
end

if ((gd~=round(gd))||(gd<0))
  if (cki); iew('decimation factor must be positive integer'); end
  gd = abs(round(gd));
end

% display args
if (dbg > 0)
  fprintf('strict=%d, elaborate=%s\n', strict, elaborate);
  fprintf('ke=%d, cl=%d, dbg=%d\n', ke, cl, dbg);
  fprintf('gp=%d, gk=%d, gn=%d gb=%d, gf=%d, gs=%d gt=%d\n', ...
          gp, gk, gn, gb, gf, gs, gt);
  fprintf('ss=%g, gd=%d\n', ss, gd);
end

% read curvature vector
k = [0 0]'; lpk = 0;
if (isfield(p,'k'))
  lpk = length(p.k);
  for i=1:min(lpk,2); k(i) = p.k(i); end
end

% read bounding curve parameters
d = [1 1 1 1 pi/4]'; lpd = 0;
if (isfield(p,'d'))
  lpd = length(p.d);
  last = 1;
  for i=1:min(lpd,5)
    if (cki && (p.d(i) <= 0))
      iee('all given boundary params must be positive');
    else d(i) = p.d(i); last = d(i);
    end
  end
  if (lpd < 4); for i=(lpd+1):4; d(i) = last; end; end
end

% read or infer surface and boundary type
if (isfield(p,'s')); st = p.s;
else
  if (lpk >= 2) % at least two curvatures given
    zk = sum(k(1:2)==0); % number of zero curvatures
    if (zk==0)
      if (sign(k(1))~=sign(k(2))); st = 'h'; else st = 'e'; end
    elseif (zk==1); st = 'y';
    else st = 'p';
    end % no nonzero curvatures
  elseif (lpk == 1); st = 'o'; % one curvature given
  else st = 'p'; % no curvatures given
  end 
  if (dbg); fprintf('inferred surface type s=%s\n from curvatures\n',st); end
  % TBD ensure inferred surface type is compatible with boundary type
end

if (isfield(p,'b')); bt = p.b;
else
  msg = 'surface type';
  switch (st)
    case {'e','h'}; bt = 'e';
    case {'y','c'}; bt = 'r';
    case {'o','s'}; bt = 'c';
    case 'p';
      msg = 'given d';
      if (lpd >= 5); bt = 'q';
      elseif (lpd == 2); bt = 'e';
      else bt = 'o'; % lpd < 2
      end
    otherwise; if (cki); iee('unknown surface type'); end
  end
  if (dbg) fprintf('inferred boundary type b=%s\n from %s\n',bt,msg); end
end

% read extrinsic pose
c = [0 0 0]'; r = [0 0 0]';
if (isfield(p,'c')); for i=1:min(length(p.c),3); c(i)=p.c(i); end; end
if (isfield(p,'r')); for i=1:min(length(p.r),3); r(i)=p.r(i); end; end

% check params and determine surface functions
switch (st)
  case 'e'; % elliptic paraboloid
    if (cki && ~strcmp('e',bt))
      iew('elliptic paraboloid surface can have only ellipse bounds');
    end
    nk = 2; kx = k(1); ky = k(2);
    if (cki && ((abs(ky)<abs(kx)) || ...
        (((kx~=0)&&(ky~=0)) && ((sign(kx)~=sign(ky))))))
      iew('wrong curvatures for elliptic paraboloid');
    end
  case 'h'; % hyperbolic paraboloid
    if (cki && ~strcmp('e',bt))
      iew('hyperbolic paraboloid surface can have only ellipse bounds');
    end
    nk = 2; kx = k(1); ky = k(2);
    if (cki && ((abs(ky)<abs(kx)) || ...
        (((kx~=0)&&(ky~=0))&&((sign(kx)~=-sign(ky))))))
      iew('wrong curvatures for hyperbolic paraboloid');
    end
  case 'y'; % cylindric paraboloid
    nk = 1; kk = k(1); k = [kk]; kx = 0; ky = kk;
    if (cki && ~strcmp('r',bt))
      iew('cylindric paraboloid surface can have only AA rect bounds');
    end
  case 'o'; % circular paraboloid
    nk = 1; kk = k(1); k = [kk]; kx = kk; ky = kk;
    if (cki && ~strcmp('c',bt))
      iew('circular paraboloid surface can have only circle bounds');
    end
  case 'p'; % plane
    nk = 0; kk = 0; k = []; kx = 0; ky = 0;
  case 's'; % sphere
    nk = 1; kk = k(1); k = [kk]; kx = kk; ky = kk;
    if (cki && ~strcmp('c',bt))
      iew('spherical surface can have only circle bounds');
    end
    if (cki && (abs(kk)*abs(d(1))>1))
      iew('bounding circle radius larger than sphere radius');
    end
    if (kk~=0)
      sl = @(x,y,z)(kk*(x.^2+y.^2+z.^2)-2*z);
      sg = @(xyz)(2*(kk*xyz-repmat([0 0 1],length(xyz),1)));
      zl = @(x,y)((1-sqrt(1-kk^2*(x.^2+y.^2)))/kk);
      ri = riquadratic(kk, kk, kk);
    end % degenerate case handled below
  case 'c'; % circular cylinder
    nk = 1; kk = k(1); k = [kk]; kx = 0; ky = kk;
    if (cki && ~strcmp('r',bt))
      iew('circular cylinder surface can have only AA rect bounds');
    end
    if (cki && (abs(kk)*abs(d(2))>1))
      iew('bounding rect height larger than cylinder radius');
    end
    if (kk~=0)
      sl = @(x,y,z)(kk*(y.^2+z.^2)-2*z);
      sg = @(xyz)(2*(xyz*diag([0 kk kk])-repmat([0 0 1],length(xyz),1)));
      zl = @(x,y)((1-sqrt(1-kk^2*y.^2))/kk);
      ri = riquadratic(0, kk, kk);
    end % degenerate case handled below
  otherwise;
    if (cki); iee('unknown surface type'); end
end

% standard curvatures
sk = [kx; ky];

% set surface functions for plane and paraboloid (sphere and circ cyl above)
if ((kx==0)&&(ky==0)) % plane
  sl = @(x,y,z)(-2*z);
  sg = @(xyz)(repmat([0 0 -2],length(xyz),1));
  zl = @(x,y)(zeros(size(x)));
  ri = riplane();
elseif (any(strcmpi(st,{'e','h','y','o'}))) % paraboloid
  sl = @(x,y,z)(kx*x.^2+ky*y.^2-2*z);
  sg = @(xyz)(2*(xyz*diag([kx ky 0])-repmat([0 0 1],length(xyz),1)));
  zl = @(x,y)((1/2)*(kx*x.^2+ky*y.^2));
  ri = riquadratic(kx, ky, 0);
end

  function nxyz = sn(xyz)
  % unit normal function
  nxyz = sg(xyz);
  gl = sqrt(sum(nxyz.*nxyz,2)); 
  gl = -1./gl; gl = [gl gl gl];
  nxyz = nxyz.*gl;
  end

% check bounds type and generate bounds params
switch (bt)
  case 'c';
    nd = 1; rr = [d(1) d(1)];
    bb = bbaaquad(rr);
    ba = baellipse(rr);
    bi = biellipse(rr);
    bs = bsellipse(rr);
    bl = blellipse(rr);
    bc = bcellipse(rr);
  case 'e';
    nd = 2;
    bb = bbaaquad(d);
    ba = baellipse(d);
    bi = biellipse(d);
    bs = bsellipse(d);
    bl = blellipse(d);
    bc = bcellipse(d);
  case 'r';
    nd = 2;
    bb = bbaaquad(d);
    ba = baaaquad(d);
    bi = biaaquad(d);
    bs = bsaaquad(d);
    bl = blaaquad(d);
    bc = bcaaquad(d);
  case 'q';
    nd = 5;
    bb = bbcquad(d);
    ba = bacquad(d);
    bi = bicquad(d);
    bs = bscquad(d);
    bl = blcquad(d);
    bc = bccquad(d);
  otherwise;
    if (cki); iee('unknown bounds type'); end
end

% display primary fields
if (dbg > 0)
  fprintf('p.s=%s, p.b=%s, nk=%d, nd=%d\n',st,bt,nk,nd);
  display(k); fprintf('kx=%g, ky=%g\n',kx,ky);
  display(d);
  display(c); display(r);
end

% generate output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (nargout() < 1); return; end

% setup output
if (cl)
  if (isfield(p,'s')); o.s = p.s; end;
  if (isfield(p,'b')); o.b = p.b; end
  if (isfield(p,'k')); o.k = p.k(1:min(length(p.k),nk)); end
  if (isfield(p,'d')); o.d = p.d(1:min(length(p.d),nd)); end
  if (isfield(p,'c')); o.c = p.c(1:min(length(p.c),3)); end
  if (isfield(p,'r')); o.r = p.r(1:min(length(p.r),3)); end
else o = p; end

  function oo = isset(o,fn)
  % check if output field is already set
  oo = (isfield(o,fn)&&~isempty(getfield(o,fn)));
  end

  function oo = iswr(o,fn)
  % check if output field should be written
  oo = (~ke||~isset(o,fn));
  if (cki&&dbg&&~oo);
    fprintf('keeping existing %s\n',fn);
  end
  end

% set defaults
wk = false; wd = false;
if (setdef)
  if (iswr(o,'s')); o.s = st; end;
  if (iswr(o,'b')); o.b = bt; end;
  wk = iswr(o,'k'); wd = iswr(o,'d');
  if (wk); o.k(1:nk) = k(1:nk); end
  if (wd); o.d(1:nd) = d(1:nd); end
  if (iswr(o,'c')); o.c(1:3) = c; end;
  if (iswr(o,'r')); o.r(1:3) = r; end
end

% generate derived %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pose
wrm = iswr(o,'rm'); wpm = iswr(o,'pm');
if (gp&&(wrm||wpm))
  rm = rexp(r); pm = [[rm c]; [0 0 0 1]];
  if (cki&&~(wrm&&wpm)); iew('gp requested but not updating both rm, pm'); end
  if (wrm); o.rm = rm; end; if (wpm); o.pm = pm; end
end

% curvatures
wkx = iswr(o,'kx'); wky = iswr(o,'ky');
if (gk&&(wkx||wky))
  if (cki&&~(wkx&&wky)); iew('gk requested but not updating both kx, ky'); end
  if (wkx); o.kx = kx; end; if (wky); o.ky = ky; end
end

% nk, nd
wnk = iswr(o,'nk'); wnd = iswr(o,'nd');
if (gn&&(wnk||wnd))
  if (cki&&~(wnk&&wnd)); iew('gn requested but not updating both nk, nd'); end
  if (wnk); o.nk = nk; end; if (wnd); o.nd = nd; end
end

% bounds
wbb = iswr(o,'bb'); wba = iswr(o,'ba');
wbi = iswr(o,'bi'); wbs = iswr(o,'bs'); wbl = iswr(o,'bl');
wbc = iswr(o,'bc');
if (gb&&(wbb||wba||wbi||wbs||wbl||wbc))
  if (cki&&~(wbb&&wba&&wbi&&wbs&&wbl&&wbc))
    iew('gb requested but not updating all of bb, ba, bi, bs, bl, bc');
  end
  if (wbb); o.bb = bb; end; if (wba); o.ba = ba; end
  if (wbi); o.bi = bi; end; if (wbs); o.bs = bs; end; if (wbl); o.bl = bl; end
  if (wbc); o.bc = bc; end
end

% surface funcs
wsl = iswr(o,'sl'); wsg = iswr(o,'sg'); wsn = iswr(o,'sn');
wzl = iswr(o,'zl'); wri = iswr(o,'ri');
if (gf&&(wsl||wsg||wsn||wzl||wri))
  if (cki&&~(wsl&&wsg&&wsn&&wzl&&wri))
    iew('gf requested but not updating all of sl, sg, sn, zl, ri');
  end
  if (wsl); o.sl = sl; end; if (wsg); o.sg = sg; end; if (wsn); o.sn = @sn; end
  if (wzl); o.zl = zl; end; if (wri); o.ri = ri; end
end

% grid samples [gx, gy, gz] and boundary samples
wgv = iswr(o,'gv'); wbv = iswr(o,'bv');
if (gs&&(wgv||wbv))

  if (cki&&~(wgv&&wbv)); iew('gs requested but not updating both gv, bv'); end

  % x (vert) and y (horiz) gridline abscissas as Nx1 mat
  gu = {[], []};
  
  % corresponding ordinates along gridline in the other dim as NxVAR cell
  % array of 1xVAR row matrices, where VAR is the number of samples along
  % that gridline (may vary from line to line)
  gvv = {{}, {}};
  
  for i=1:2 %i=1 x gridlines (vertical), i=2 y gridlines (horizontal)
    
    b = bb(i,:); %[min max] bounds in dimension i
    
    % abscissas in this dim as Nx1 matrix
    gu{i} = samples(b(1),b(2),ss,0,0); %sample open interval
    
    % [min, max] ordinates in other dim as Nx2 matrix
    mm = bi(i,gu{i});
    
    % valid gridline indices in this dim
    vi = ~any(isnan(mm)|chkimag(mm),2);
    % skip any invalid gridlines
    gu{i} = gu{i}(vi); mm = real(mm(vi,:));
    
    % Generate itermediate ordinates in the other dim. Can skip if zero
    % curvature in other dim, but still must convert to cell array.
    n = length(gu{i});
    j = imod(i+1,2); % other dim
    if (sk(j)~=0) % other dim curvature not zero
      gvv{i} = cell(n,1);
      % sample closed interval
      for l=1:n; gvv{i}{l} = samples(mm(l,1),mm(l,2),ss,1,1); end
    else gvv{i} = mat2cell(mm,ones(n,1),[2]); end % other dim curvature zero
  end
  
  % number of gridlines
  ngx = length(gu{1}); ngy = length(gu{2}); ng = ngx+ngy;
  
  % Nx1 arrays of vertex coords, one triple per gridline
  gx = cell(ng,1); gy = cell(ng,1); gz = cell(ng,1);
  
  l = 1; mv = 0;
  for i=1:2 %i=1 x gridlines (vertical), i=2 y gridlines (horizontal)
    for j=1:length(gu{i}) % for each abscissa
      u = repmat(gu{i}(j),length(gvv{i}{j}),1); % abscissas all same along line
      v = gvv{i}{j}'; % ordinates along line
      if (i==1); gx{l} = u; gy{l} = v; else gx{l} = v; gy{l} = u; end
      gz{l} = zl(gx{l},gy{l});
      vi = ~isnan(gz{l})&~chkimag(gz{l});
      gx{l} = gx{l}(vi); gy{l} = gy{l}(vi); gz{l} = real(gz{l}(vi));
      mv = max(mv,length(gx{l}));
      l = l+1;
    end
  end
  
  if (gd<2); gdi = 1:ng;
  else % decimate grid
    gss = ss*gd;
    for i=1:2
      ngui = length(gu{i});
      first = 1+ceil(gu{i}(1)/gss)*gd-round(gu{i}(1)/ss);
      last = ngui-(round(gu{i}(ngui)/ss)-floor(gu{i}(ngui)/gss)*gd);
      gdii{i} = first:gd:last;
    end
    gdi = [gdii{1}, ngx+gdii{2}];
    ng = length(gdi);
  end
  
  gxd = gx(gdi); gyd = gy(gdi); gzd = gz(gdi);
  
  if (wgv); o.gv = {gxd, gyd, gzd}; end
  
  % boundary samples
  bv2 = bs(ss,kx,ky);
  bx = [bv2(:,1); bv2(1,1)];
  by = [bv2(:,2); bv2(1,2)];
  bz = zl(bx,by);
  bv3 = [bx, by, bz];
  vi = ~any(isnan(bv3)|chkimag(bv3),2);

  if (wbv); o.bv = real(bv3(vi,:)); end

end % samples

% triangulation
wfv = iswr(o,'fv'); wft = iswr(o,'ft');
if (gt&&(wfv||wft))

  if (cki&&~(wfv&&wft)); iew('gt requested but not updating both fv, ft'); end

  % delaunay triangulation of all un-decimated grid samples

  % for quads append boundary verts to list as they may not be grid samples
  % (ellipse and circle extrema are always grid samples)
  qx = []; qy = []; qz = [];
  if (strcmp('r',bt)||strcmp('q',bt));
    bvv = bs(ss,0,0); % force curvatures to zero to only get quad vertices
    qx = bvv(:,1); qy = bvv(:,2); qz = zl(qx,qy);
  end
  fx = [cat(1,gx{:}); qx];
  fy = [cat(1,gy{:}); qy];
  fz = [cat(1,gz{:}); qz];
  [v2,vi] = unique([fx,fy],'rows');
  fx = v2(:,1); fy = v2(:,2); fz = fz(vi);
  fi = delaunay(fx,fy);

  if (wfv); o.fv = [fx, fy, fz]; end
  if (wft); o.ft = fi; end
  
end % generate faces

% triangulation area (Heron's formula)
if (ga&&iswr(o,'ta'))
  a = sqrt(sum((o.fv(o.ft(:,2),:)-o.fv(o.ft(:,1),:)).^2,2)); % 1-c-3
  b = sqrt(sum((o.fv(o.ft(:,3),:)-o.fv(o.ft(:,2),:)).^2,2)); % |  / 
  c = sqrt(sum((o.fv(o.ft(:,1),:)-o.fv(o.ft(:,3),:)).^2,2)); % a b  
  s = 0.5*(a+b+c);                                           % |/   
  o.ta = sum(sqrt(s.*(s-a).*(s-b).*(s-c)));                  % 2    
end

% write ss and gd
if (iswr(o,'ss')); o.ss = ss;
elseif (cki&&(ss~=o.ss)); iew('not writing updated ss'); end

if (iswr(o,'gd')); o.gd = gd;
elseif (cki&&(gd~=o.gd)); iew('not writing updated gd'); end

% display some derived fields
if (dbg > 0)
  if (gp); display(pm); end
  display(sl); display(zl);
  display(bb); display(ba); display(bi); display(bs);
end

if (dbg); fprintf('patchchk: %gs\n',toc(tstart)); end

end % patchchk()

% helper functions

function ba = baellipse(d)
% generate ellipse boundary area
ba = pi*d(1)*d(2);
end

function bi = biellipse(d)
% generate ellipse boundary intersection function
  function o = f(i,u)
  % o=[a b] are the min, max ordinates of the intersection of
  % vert (i=1) or horiz (i=2) line at abscissa u
  o = nan(length(u),2);
  j = imod(i+1,2); % other dim
  vi = (abs(u)<=d(i)); % indices of valid intersections
  o(vi,2) = (d(j)/d(i))*sqrt(d(i)^2-u(vi).^2);
  o(vi,1) = -o(vi,2); % always symmetric
  end
bi = @f;
end

function bs = bsellipse(d)
% generate ellipse boundary sample function
  function o = f(ss,kx,ky)
  % Overall approach: compute points in quadrant I
  % (CCW angle > 0 and <= pi/2) then copy four times with rotations
  % of pi/2
  %
  % Sample points are calculated at the intersections of the ellipse
  % with lines of an axis-aligned grid with spacing ss
  %
  % The output points are unique and CCW ordered starting with a
  % sample along the positive x axis
  
  % For each dim i: u{i} = sample abscissas, v{i} = ordinate,
  % p{i} = CCW angle
  for i=1:2
    u{i} = [0:ss:d(i)]'; % generate abscissas
    j = imod(i+1,2); % other dim
    v{i} = (d(j)/d(i))*sqrt(d(i)^2-u{i}.^2); % generate ordinates
    if (i == 1); x = u{i}; y = v{i}; else y = u{i}; x = v{i}; end
    p{i} = atan2(y,x); % calculate angles
  end
  % now extract only unique vertices in CCW order
  pp = [p{1}; p{2}]; xx = [u{1}; v{2}]; yy = [v{1}; u{2}];
  [~,i] = unique(pp); x = xx(i); y = yy(i); n = size(i);
  q1 =[x y];
  q2 = flipud(q1*[-1,0;0, 1]);
  q3 =        q1*[-1,0;0,-1];
  q4 = flipud(q1*[ 1,0;0,-1]);
  o = [q1(2:n,:); q2(2:n,:); q3(2:n,:); q4(2:n,:)];
  end
bs = @f;
end

function bl = blellipse(d)
% generate implicit boundary function for ellipse
rx2 = d(1)*d(1); ry2 = d(2)*d(2);
  function o = f(x,y); o = x.*x*ry2+y.*y*rx2-rx2*ry2; end
bl = @f;
end

function bc = bcellipse(d)
% generate squre/boundary intersection area function for ellipse
% uses secant approximation, requires x, y to be integer
a = d(1); b = d(2); bl = blellipse(d);
  function o = f(x,y,w)

  assert(isequal(x,round(x)));
  assert(isequal(y,round(y)));

  % flip quadrants II, III, IV to I
  ii = x<0; x(ii) = -x(ii)-1; ii = y<0; y(ii) = -y(ii)-1;

  x = x*w; y = y*w; xw = x+w; yw = y+w; % convert from units of w

  % bool indices of square corners inside the ellipse
  llci = bl(x,y)<=0; lrci = bl(xw,y)<=0;
  urci = bl(xw,yw)<=0; ulci = bl(x,yw)<=0;

  % intersection area by case analysis, see Kanoulas, Vona ICRA2013

  X = @(y)(a*sqrt(1-y.*y/(b*b)));
  Y = @(x)(b*sqrt(1-x.*x/(a*a)));

  Xy = X(y); Yx = Y(x); Xyw = X(yw); Yxw = Y(xw); YXyw = Y(Xyw);

  a1 = (Xyw-x)*w+(xw-Xyw).*(YXyw-y)+(xw-Xyw).*(y+w-YXyw)/2;
  a2 = (xw-x).*(Yxw-y)+(xw-x).*(Yx-Yxw)/2;
  a3 = (yw-y).*(Xyw-x)+(yw-y).*(Xy-Xyw)/2;
  a4 = (Xy-x).*(Yx-y)/2;

  o = zeros(size(x));

  ii = urci;                  o(ii) = w*w; 
  %ii = ~llci;                o(ii) = 0; % leave at zero
  ii = llci&~urci&lrci&ulci;  o(ii) = a1(ii);
  ii = llci&lrci&~ulci;       o(ii) = a2(ii);
  ii = llci&ulci&~lrci;       o(ii) = a3(ii);
  ii = llci&~lrci&~ulci;      o(ii) = a4(ii);

end
bc = @f;
end

function bb = bbaaquad(d)
bb = [-d(1), d(1); -d(2), d(2)];
end

function ba = baaaquad(d)
% generate axis-aligned quadrilateral boundary area
ba = 4*d(1)*d(2);
end

function bi = biaaquad(d)
% generate axis-aligned quadrilateral boundary intersection function
  function o = f(i,u)
  % o=[a b] are the min, max ordinates of the intersection of
  % vert (i=1) or horiz (i=2) line at abscissa u
  o = nan(length(u),2);
  j = imod(i+1,2); % other dim
  vi = (abs(u)<=d(i)); % indices of valid intersections
  o(vi,:) = repmat([-d(j), d(j)],length(u),1); % always symmetric
  end
bi = @f;
end

function bs = bsaaquad(d)
% generate axis-aligned quadrilateral boundary sample function
rx = d(1); ry = d(2);
bs = bsquad([rx, ry; -rx, ry; -rx, -ry; rx, -ry]);
end

function bl = blaaquad(d)
% generate implicit boundary function for axis aligned quad
rx = d(1); ry = d(2);
bl = blquad([rx, ry; -rx, ry; -rx, -ry; rx, -ry]);
end

function bc = bcaaquad(d)
% generate squre/boundary intersection area function for axis aligned quad
% exact, can handle non-integer x, y
a = d(1); b = d(2);
  function o = f(x,y,w)

  %assert(isequal(x,round(x)));
  %assert(isequal(y,round(y)));

  % flip quadrants II, III, IV to I
  ii = x<0; x(ii) = -x(ii)-1; ii = y<0; y(ii) = -y(ii)-1;

  x = x*w; y = y*w; xw = x+w; yw = y+w; % convert from units of w

  o = zeros(size(x));

  % bool indices of square corners inside the quad
  xi = abs(x)<=a; yi = abs(y)<=b; xwi = abs(xw)<=a; ywi = abs(yw)<=b;
  llci = xi&yi; lrci = xwi&yi; urci = xwi&ywi; ulci = xi&ywi;

  % intersection area by case analysis, see Kanoulas, Vona ICRA2013

  a5 = w*(b-y); a6 = w*(a-x); a7 = (a-x).*(b-y);

  ii = urci;             o(ii) = w*w; 
  %ii = ~llci;           o(ii) = 0; % leave at zero
  ii = llci&lrci&~ulci;  o(ii) = a5(ii);
  ii = llci&ulci&~lrci;  o(ii) = a6(ii);
  ii = llci&~lrci&~ulci; o(ii) = a7(ii);

  end
bc = @f;
end

function [bb, v] = bbcquad(d)
% generate convex quadrilateral bounding box function
%
% also outputs quad verts v 4x2 in CCW order starting in quadrant 1
g = d(5); p = [g; pi-g; pi+g; -g]; r = [d(1); d(2); d(3); d(4)];
v = [cos(p).*r, sin(p).*r];
bb = [inf, -inf; inf, -inf];
for i=1:4
  bb(:,1) = min(bb(:,1),v(i,:)');
  bb(:,2) = max(bb(:,2),v(i,:)');
end
end

function ba = bacquad(d)
% generate convex quadrilateral boundary area
% sin((2*pi-4*d(5))/2) = sin(pi-2*d(5)) = sin(2*d(5))
% sin(2*d(5)) = 2*sin(d(5))*cos(d(5))
ba = ((d(1)*d(4)+d(2)*d(3))+(d(1)*d(2)+d(3)*d(4)))*sin(d(5))*cos(d(5));
end

function bi = bicquad(d)
% generate convex quadrilateral boundary intersection function
[bb, v] = bbcquad(d);
  function o = f(i,u)
  % o=[n x] are the min, max ordinates of the intersection of
  % vert (i=1) or horiz (i=2) line at abscissa u
  o = nan(length(u),2);
  vi = ((u>=bb(i,1))&(u<=bb(i,2))); % indices of valid intersections
  o(vi,:) = lqinter(v,i,u(vi));
  end
bi = @f;
end

function o = lqinter(v,i,u)
% intersection Nx2 of convex quad with ccw verts v with vertical (i=1) or
% horizontal (i=2) line at abscissa u Nx1
o = repmat([nan, nan],length(u),1); % find min and max intersections
for j=1:4 % check all four boundary segments
  w = llinter(v(j,:),v(imod(j+1,4),:),i,u); % nan if no intersection
  o(:,1) = min(o(:,1),w); o(:,2) = max(o(:,2),w); % min(nan,1) = 1
end
end

function v = llinter(a,b,i,u)
% intersection Nx2 of segment ab with vert (i=1) or horiz (i=2) line at
% abscissa u Nx1
v = nan(length(u),1); % no intersection
d = b(i)-a(i);
if (d~=0) % not parallel?
  j = imod(i+1,2); % other dim
  vi = ((u>=min(a(i),b(i)))&(u<=max(a(i),b(i)))); % abscissa in bounds?
  v(vi) = ((b(j)-a(j))*u(vi)-a(i)*b(j)+a(j)*b(i))/d;
end
end

function bs = bscquad(d)
% generate convex quadrilateral boundary sample function
g = d(5); p = [g; pi-g; pi+g; -g]; r = [d(1); d(2); d(3); d(4)];
bs = bsquad([cos(p).*r, sin(p).*r]);
end

function bs = bsquad(v)
% generate quadrilateral boundary sample function
% v are 4 quad verts in CCW order
% k=[kx ky] are patch curvatures
%
% impl is a little tricky because it's designed to be robust for non-aa
% quads with curvature, even though non-aa quad bounds may only strictly
% be allowed for planes

  function o = f(ss,kx,ky)
  k = [kx; ky];
  if (~any(k)); o = v; % shortcut if no curvature
  else
    % 1x5 cell array of boundary samples, start plus one Nx2 array
    % per edge
    es{1} = v(1,:); % start
    for i=1:4 % for each boundary segment
      s = v(i,:); e = v(imod(i+1,4),:); % segment start and end points
      d = e-s; % difference vector
      if (~any(d(:).*k(:))); es{i+1} = e; % shortcut, no curvature in chngng dim
      else
        for j=1:2 % for each dim
          k = imod(j+1,2); % other dim
          if (d(j)==0);
            uu{j}=[s(j),e(j)]; vv{j}=[s(k),e(k)]; % shortcut
          else
            uu{j} = samples(s(j),e(j),ss*sign(d(j)),1,1); % abscissas
            nu = length(uu{j});
            assert(nu>=2); % this happens...
            if (nu>2)
              v2 = llinter(s,e,j,uu{j}(2));
              if (nu>3)
                v3 = llinter(s,e,j,uu{j}(nu-1));
                vv{j}=[s(k),linspace(v2,v3,nu-2),e(k)];
              else vv{j}=[s(k),v2,e(k)];
              end % length(uu{j})==3
            else vv{j}=[s(k),e(k)]; % length(uu{j})==2
            end
            assert(nu==length(vv{j}));
          end
        end % for each dim
        x = [uu{1}(:); vv{2}(:)];
        y = [vv{1}(:); uu{2}(:)];
        d2 = (x-s(1)).^2+(y-s(2)).^2;
        [~,vi] = unique(d2);
        xy = [x,y];
        es{i+1} = xy(vi,:);
      end
    end % for each boundary segment
    o = [es{1}; es{2}; es{3}; es{4}; es{5}];
  end
  end %f(ss,k)
bs = @f;
end

function bl = blcquad(d)
% generate implicit boundary function for convex quad
g = d(5); p = [g; pi-g; pi+g; -g]; r = [d(1); d(2); d(3); d(4)];
bl = blquad([cos(p).*r, sin(p).*r]);
end

function bc = bccquad(d)
% generate squre/boundary intersection area function for convex quad
% exact, can handle non-integer x, y
  [bbq, vq] = bbcquad(d); blq = blquad(vq);
  function o = f(x,y,w)

  %assert(isequal(x,round(x)));
  %assert(isequal(y,round(y)));

  % do not flip quadrants

  sz = size(x); ns = prod(sz);

  % convert from units of w and vectorize to NSx1
  x = x(:)*w; y = y(:)*w; xw = x+w; yw = y+w; oov = ones(ns,1);

  assert(length(x)==length(y));

  % coords of square corners NSx1
  llcx = x; llcy = y; lrcx = xw; lrcy = y;
  urcx = xw; urcy = yw; ulcx = x; ulcy = yw;

  % bool indices of square corners inside quad NSx1
  llci = blq(llcx, llcy)<=0; lrci = blq(lrcx, lrcy)<=0;
  urci = blq(urcx, urcy)<=0; ulci = blq(ulcx, ulcy)<=0;

  % coords of quad corners 1x1
  vq1x = vq(1,1); vq1y = vq(1,2); vq2x = vq(2,1); vq2y = vq(2,2);
  vq3x = vq(3,1); vq3y = vq(3,2); vq4x = vq(4,1); vq4y = vq(4,2);

  % bool indices of quad corners inside square NSx1
  vq1i = (x<=vq1x)&(vq1x<=xw)&(y<=vq1y)&(vq1y<=yw);
  vq2i = (x<=vq2x)&(vq2x<=xw)&(y<=vq2y)&(vq2y<=yw);
  vq3i = (x<=vq3x)&(vq3x<=xw)&(y<=vq3y)&(vq3y<=yw);
  vq4i = (x<=vq4x)&(vq4x<=xw)&(y<=vq4y)&(vq4y<=yw);

  % coords of quad corners NSx1
  vq1x = vq1x*oov; vq1y = vq1y*oov; vq2x = vq2x*oov; vq2y = vq2y*oov;
  vq3x = vq3x*oov; vq3y = vq3y*oov; vq4x = vq4x*oov; vq4y = vq4y*oov;

  % [xmin, xmax] intersections of quad with square bottom, top NSx1
  bx = lqinter(vq,2,y); tx = lqinter(vq,2,yw);
  bxl = bx(:,1); bxr = bx(:,2); txl = tx(:,1); txr = tx(:,2);

  % [ymin, ymax] intersections of quad with square left, right NSx1
  ly = lqinter(vq,1,x); ry = lqinter(vq,1,xw);
  lyb = ly(:,1); lyt = ly(:,2); ryb = ry(:,1); ryt = ry(:,2);

  % bool indices of valid intersections NSx1
  bxli = (x<=bxl)&(bxl<=xw); bxri = (x<=bxr)&(bxr<=xw);
  txli = (x<=txl)&(txl<=xw); txri = (x<=txr)&(txr<=xw);
  lybi = (y<=lyb)&(lyb<=yw); lyti = (y<=lyt)&(lyt<=yw);
  rybi = (y<=ryb)&(ryb<=yw); ryti = (y<=ryt)&(ryt<=yw);

  % coords of the 16 candidate verts NSx16
  xx = [llcx lrcx urcx ulcx vq1x vq2x vq3x vq4x bxl bxr txl txr x x xw xw];
  yy = [llcy lrcy urcy ulcy vq1y vq2y vq3y vq4y y y yw yw lyb lyt ryb ryt];

  % valid indices of the candidates NSx16
  ii = [llci lrci urci ulci vq1i vq2i vq3i vq4i ...
        bxli bxri txli txri lybi lyti rybi ryti];

  ni = sum(ii,2); % number of valid verts per polygon NSx1

  xx(~ii) = nan; yy(~ii) = nan; % mark invalid verts

  xm = nanmean(xx,2); ym = nanmean(yy,2); % polygon center coords NSx1

  xx = xx-repmat(xm,1,16); yy = yy-repmat(ym,1,16); % recenter polys

  aa = atan2(yy,xx); % angle of each vert NSx16

  % poly verts in ccw order
  [aa,ii] = sort(aa,2);

  %xx = xx(ii); yy = yy(ii); % unfortunately this does not work

  % but this is supposed to be fast with JIT
  for r = 1:ns
    xx(r,:) = xx(r,ii(r,:));
    yy(r,:) = yy(r,ii(r,:));
  end

  rr = sqrt(yy.*yy+xx.*xx); % radius of each vert NSx1

  % column of last valid vert of each poly (1 if none) NSx1
  nii = ni; nii(nii==0) = 1;

  % angle and radius of last valid vert of each poly NSx1
  aal = aa((nii-1)*ns+(1:ns)'); aal = aal-2*pi;
  rrl = rr((nii-1)*ns+(1:ns)'); % lindex = (col-1)*numrows+row

  aap = [aal aa(:,1:15)]; % angle of previous vert NSx16
  rrp = [rrl rr(:,1:15)]; % radius of previous vert NSx16

  ta = 0.5*rr.*rrp.*sin(aa-aap); % up to 16 triangle areas per polygon NSx16

  pa = nansum(ta,2); pa(isnan(pa)) = 0; % polygon areas NSx1

  o = reshape(pa,sz);

  end
bc = @f;
end

function bl = blquad(v)
% generate implicit boundary function for quadrilateral
% v are 4 quad verts in CCW order (4x2)

% edge direction vectors
d = [v(2,:)-v(1,:); v(3,:)-v(2,:); v(4,:)-v(3,:); v(1,:)-v(4,:)];
l = [norm(d(1,:)); norm(d(2,:)); norm(d(3,:)); norm(d(4,:))];
vi = (l>eps); ns = sum(vi);

% TBD for now don't get crazy with degenerate cases
if (ns<3); bl = @(x,y)(sqrt(x.*x+y.*y)); return; end

% make s and d are the start points and unit direction vectors of each
% non-degenerate side
s = zeros(ns,2); dd = zeros(ns,2); j = 1;
for i=1:4
  if (vi(i)); s(j,:) = v(i,:); dd(j,:) = d(i,:)/l(i); j = j+1; end;
end
d = dd;

n = zeros(ns,2); % outward pointing unit normals
for i=1:ns; nn = cross([d(i,:),0],[0,0,1]); n(i,:) = nn(1,1:2); end

c = zeros(ns,1); % perp dist to origin
for i=1:ns; c(i) = -n(i,:)*s(i,:)'; end

  function o = f(x,y)
  sz = size(x); nd = sz(1)*sz(2);
  x = x(:)'; y = y(:)'; % always row vectors
  d = zeros(ns,nd); % distance from each pt (cols) to each line (rows)
  for i=1:ns; d(i,:) = x*n(i,1)+y*n(i,2)+c(i); end
  o = zeros(1,nd);
  % for pts inside, return closest negative distance
  ni = (d<0); vi = all(ni); dd = max(d); o(vi) = dd(vi);
  % for pts outside, return closest positive distance
  d(ni) = inf; dd = min(d); vi = ~vi; o(vi) = dd(vi);
  o = reshape(o,sz);
  end

bl = @f;

end % blquad

function p = samples(s,e,ss,cs,ce)
% generate samples from s to e, always at integer multiples of ss
% s is always included (even if not an integer multiple of ss) if cs=1
% e is always included (even if not an integer multiple of ss) if ce=1
% s is never included (even if an integer multiple of ss) if cs=0
% e is never included (even if an integer multiple of ss) if ce=0
first = fix(s/ss)*ss; % first sample generated by colon
last = first+fix((e-first)/ss)*ss; % last sample generated by colon
if (~cs&&(first==s)); first = first+ss; end
if (~ce&&(last==e)); last = last-ss; end
p = [];
if (abs(last-first)>=0); p = first:ss:last; end
sb = []; eb = []; % bounds to possibly append, default none
if (cs&&(first~=s)); sb = [s]; end
if (ce&&(last~=e)); eb = [e]; end
if ((~isempty(sb))||(~isempty(eb))) p = [sb, p, eb]; end
end

function j = imod(i,d)
% modular arithmetic helper for base-one indices
j = mod(i-1,d)+1;
end

function o = chkimag(x)
% check if the imaginary part of x is more than a small tolerance
imagtol = 1e-6;
o = abs(imag(x)) > imagtol;
end

function ri = riplane()
% generate range intersection function for planes
  function r = f(mx,my,mz,cx,cy,cz); r = -cz./mz; end
ri = @f;
end

function ri = riquadratic(kx, ky, kz)
% generate quadratic form range intersection function

kk = diag([kx, ky, kz]);
  function r = f(mx,my,mz,cx,cy,cz)
  sz = size(mx); nd = sz(1)*sz(2);
  
  mm = [mx(:)'; my(:)'; mz(:)'];
  
  cc = [cx(:)'; cy(:)'; cz(:)'];
  
  zz = repmat([0 0 1]',1,nd);
  
  a = sum(mm.*(kk*mm));
  b = 2*sum(mm.*(kk*cc-zz));
  c = sum(cc.*(kk*cc-2*zz));
  
  [r1, r2] = quadsol(a, b, c);
  
  % discard complex solutions
  r1(imag(r1)~=0) = nan; r2(imag(r2)~=0) = nan;
  
  % for sphere and circ cyl, discard "top half" intersections
  if (kz~=0)
    kz1 = kz*(cc(3,:)+mm(3,:).*r1); kz2 = kz*(cc(3,:)+mm(3,:).*r2);
    r1(kz1>1) = nan; r2(kz2>1) = nan;
  end
  r = inf*ones(sz); % default intersections to infinity
  % if both solutions are nonnegative, select the smaller one
  ii = (r1>=0)&(r1<r2); r(ii) = r1(ii);
  ii = (r2>=0)&(r2<r1); r(ii) = r2(ii);
  % if one solution is positive and the other negative,
  % select the positive one
  ii = (r1>0)&(r2<0); r(ii) = r1(ii);
  ii = (r2>0)&(r1<0); r(ii) = r2(ii);
  % if both solutions are negative, select the one closer to zero
  ii = (r1<0)&(r2<r1); r(ii) = r1(ii);
  ii = (r2<0)&(r1<r2); r(ii) = r2(ii);
  % if only one valid solution, take it
  ii = ~isnan(r1)&isnan(r2); r(ii) = r1(ii);
  ii = ~isnan(r2)&isnan(r1); r(ii) = r2(ii);
  r = reshape(r,sz);
  end

ri = @f;

end
