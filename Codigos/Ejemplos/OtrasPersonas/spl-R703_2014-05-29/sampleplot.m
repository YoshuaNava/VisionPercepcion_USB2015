function h = sampleplot(varargin)
% sampleplot(...) plots xyz or range samples and/or mesh
%
%   The plot is made in the current axes of the current figure, if any, else a
%   new figure window is opened.  The return value h is an hggroup handle;
%   the generated graphics are its children.
%
%   All arguments must be given as name,value pairs. Unrecognized names cause
%   warnings.  If a name is given more than once the last-given (rightmost)
%   value takes precedence.
%
%   Arguments 'x', 'y', 'z' must either be all present or all absent.  Mutually
%   exclusive with the group 'mx', 'my', 'mz', 'r'.  If present, the input is
%   xyz data; the values of each must be arrays of the same size MxN.
%
%   Arguments 'mx', 'my', 'mz', 'r' must either be all present or all absent.
%   Mutually exclusive with the group 'x', 'y', 'z'.  If present the input is
%   range data; the values of each must be arrays of the same size MxN.
%
%   OPTIONS
%
%   Options 'cx', 'cy', 'cz' (default 0): coordinates of measurement ray
%   starting points for range data.  May each either be scalar or all MxN.
%
%   Option 'dp' (default 1): whether to draw sample points
%
%   Option 'df' (default 0): whether to draw sample mesh faces, which is only
%   allowed when equal sized xyz data larger than 2x2 is supplied 
%
%   Option 'ps' (default 6): size of point markers in units.
%   points.
%
%   Option 'pc' (default 'b'): point color, either a colorspec or [r g b].
%
%   Option 'ec', 'fc' (default 'c', 'none'): edge and face color when
%   rendering mesh faces, either a colorspec or [r g b].
%
%   Options 'el', 'fl' (default 'flat', 'flat'): edge and face lighting when
%   rendering mesh faces.  One of 'none', 'flat', 'gouraud', 'phong'.
%
%   Option 'fa', 'ea' (default 1): edge and face alpha when rendering mesh
%   faces.
%
%   Option 'lw' (default 0.5): edge width when rendering mesh faces in points.
%
%   Option 'pt' (default '.'): point marker type.
%
%   Option 'frustum' (default []): ignored if empty; otherwise must be either
%   1xF or Fx1 where F is 8 or 10, giving camera frustum model parameters in the
%   order [fh, fv, nh, nv, fpx, fpy, fpz, fux, fuy, fuz].  fh and fv are the
%   horizontal and vertical fields of view in radians, nh and nv are either
%   absent or give the number of horizontal and vertical pixels, [fpx; fpy; fpz]
%   is the view frustum pointing vector (the depth of the drawn frustum is taken
%   as the length of this vector), and [fux; fuy; fuz] is the frustum "up"
%   vector (length ignored).  The camera center of projection is [cx; cy; cz] if
%   given; it is an error to supply both a nonempty frustum and any nonscalar
%   cx, cy, or cz.
%
%   Option 'uw' (default 0.5): frustum line width in points
%
%   Option 'uc' (default 'm'): frustum line color, either a scalar color name
%   or [r g b].
%
%   Option 'dr' (default 0): whether to draw measurement rays
%
%   Option 'rw' (default 0.5): measurement ray line width in points
%
%   Option 'rc' (default 'g'): measurement ray line color, either a scalar color
%   name or [r g b].
%
%   Option 'dx' (default 0): whether to draw sample pixels, requires frustum
%   with nh and nv
%
%   Option 'xw' (default 0.5): sample pixel line width in points
%
%   Option 'xc' (default 'm'): sample pixel line color, either a scalar color
%   name or [r g b].
%
%   Option 'da' (default 1): whether to draw RGB axes triad for the frustum
%
%   Option 'aw' (default 0.5): axes triad line width in points
%
%   Option 'as' (default 1): axes triad scale
%
%   Options 'ag', 'ar', 'av', 'kv', 'gr': see axescfg(), applies when
%   'parent' is nonempty
%
%   Option 'fb' (default 0): whether to fit axes limits to all axes contents
%   (not just the new contents) with fitbbox()
%
%   Option 'parent' (default []): if nonempty then this is the parent for all
%   generated hgtransforms
%
%   Option 'strict' (default 0): whether to issue a warning if any input was
%   nan, inf, or complex (such points are always skipped)
%
%   Options 'subsample', 'decimate', 'lowres': see samplecvt(); subsample is
%   not allowed when drawing faces.
%
%   Option 'adjtri' (default []): organized triangle mesh in the adjacency
%   format described in samplemesh.m used to mask the displayed triangles.  If
%   given the dimensions of adjtri must match the dimensions of the vertex
%   coordinate data after applying any decimate or lowres options.
%
%   Option 'zoom' (default 0): if positive then call camzoom() on the axes
%   with this zoom factor
%
%   Option 'dbg' (default 0): may have any nonnegative value.  dbg=0 disables
%   debug.  Larger values enable successively more verbose debug.
%
% Copyright (C) 2013 Marsette A. Vona

tstart = tic();

smo = {'otype','cleanfaces'}; % samplemesh options

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

ixyz = 0; % set if input is detected as xyz
irng = 0; % set if input is detected as range

% option defaults
ix = []; iy = []; iz = []; ir = [];
cx = []; cy = []; cz = [];
dp = 1; df = 0; ps = 6; pc = 'b'; pt = '.';
ec = 'c'; fc = 'none';
fa = 1; ea = 1;
el = 'flat'; fl = 'flat';
lw = 0.5;
frustum = [];
uw = 0.5; uc = 'm';
dr = 0; rw = 0.5; rc = 'g';
dx = 0; xw = 0.5; xc = 'g';
da = 1; aw = 0.5; as = 1;
ag = 0; ar = 1; av = 0; kv = 0; gr = 1; fb = 0;
strict = 0;
subsample = 0; decimate = 0; lowres = 0;
adjtri = [];
parent = [];
zoom = 0;
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'x'; ix = v; ixyz = 1;
      case 'y'; iy = v; ixyz = 1;
      case 'z'; iz = v; ixyz = 1;
      case 'mx'; ix = v; irng = 1;
      case 'my'; iy = v; irng = 1;
      case 'mz'; iz = v; irng = 1;
      case 'r'; ir = v; irng = 1;
      case 'cx'; cx = v; case 'cy'; cy = v; case 'cz'; cz = v;
      case 'dp'; dp = v; case 'df'; df = v;
      case 'ps'; ps = v; case 'pc'; pc = v; case 'pt'; pt = v;
      case 'ec'; ec = v; case 'fc'; fc = v; 
      case 'ea'; ea = v; case 'fa'; fa = v;
      case 'el'; el = v; case 'fl'; fl = v;
      case 'lw'; lw = v;
      case 'frustum'; frustum = v;
      case 'uw'; uw = v; case 'uc'; uc = v;
      case 'dr'; dr = v; case 'rw'; rw = v; case 'rc'; rc = v;
      case 'dx'; dx = v; case 'xw'; xw = v; case 'xc'; xc = v;
      case 'ag'; ag = v; case 'ar'; ar = v; case 'av'; av = v;
      case 'kv'; kv = v; case 'gr'; gr = v; case 'fb'; fb = v;
      case 'da'; da = v; case 'aw'; aw = v; case 'as'; as = v;
      case 'parent'; parent = v;
      case 'strict'; strict = v;
      case 'subsample'; subsample = v; case 'decimate'; decimate = v;
      case 'lowres'; lowres = v;
      case 'adjtri'; adjtri = v;
      case 'zoom'; zoom = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

icv = ((~isscalar(cx))||(~isscalar(cy))||(~isscalar(cz)));

% calculate xyz data
cvtopts = {'decimate',decimate,'subsample',subsample,'lowres',lowres};
if (ixyz); cvtopts = {cvtopts{:},'x',ix,'y',iy,'z',iz};
elseif (irng)
  cvtopts = {cvtopts{:},'mx',ix,'my',iy,'mz',iz,'r',ir,'cx',cx,'cy',cy,'cz',cz};
else error('must give either x, y, z, or mx, my, mz, r');
end

nd = length(ix(:));
[x y z] = samplecvt(cvtopts{:});

if (strict&&(nd<length(x)))
  warning('some input data was complex, nan, or inf');
end

% get axes
if (isempty(parent))
  a = newplot(); % gets current or makes new, as appropriate
  washold = ishold(); hold('on');
end

% create group
h = hggroup();
set(h,'Tag','sampleplot');
if (~isempty(parent)); set(h,'Parent',parent); end

% plot points
if (dp)
  % it seems that scattergroup data *must* be vectorized, so keep track of
  % the original matrix dims in UserData
  hs = scatter3(x(:),y(:),z(:),'Tag','samples','UserData',size(x),...
                'Marker',pt,'MarkerFaceColor',pc,'MarkerEdgeColor',pc,...
                'SizeData', ps*ps); % needs pts^2
  set(hs,'Parent',h); % works here but not when given in arglist above
end

% plot mesh
if (df)

  if (~ixyz); error('must have xyz input data to plot faces'); end
  if (subsample); error('cannot subsample when plotting faces'); end

  sx = size(ix); sy = size(iy); sz = size(iz);
  if (~isequal(sx,sy,sz))
    error('must have equal size input data to draw faces');
  end

  % reconvert without cleaning to keep grid organization
  if (decimate||lowres)
    [xx yy zz] = samplecvt('x',ix,'y',iy,'z',iz,...
                           'decimate',decimate,'lowres',lowres);
  else xx = ix; yy = iy; zz = iz; end
  
  sx = size(xx); nr = sx(1); nc = sx(2);
  if ((nr<2)||(nc<2))
    error('data must be > 2x2 (after decimation and lowres) to draw faces');
  end

  % add given adjtri to samplemesh opts to filter the mesh
  if (~isempty(adjtri))
    sa = size(adjtri);
    if (isequal(sx,sa)); smo = [smo {'adjtri',adjtri}];
    else error('adjtri %dx%d but vertices %dx%d',sa(1),sa(2),nr,nc); end
  end

  % make a mesh in cleanfaces format for patch()
  ff = samplemesh('x',xx,'y',yy,'z',zz,smo{:});

  vv = [xx(:),yy(:),zz(:)];

  patch('Parent',h,'Tag','faces',...
        'Vertices',vv,'Faces',ff,...
        'EdgeColor',ec,'FaceColor',fc,'EdgeAlpha',ea,'FaceAlpha',fa,...
        'EdgeLighting',el,'FaceLighting',fl,...
        'LineWidth',lw,...
        'SpecularStrength',0.2,'DiffuseStrength',0.5);

end

% plot rays
if (dr)
  cxx = cx(:); cyy = cy(:); czz = cz(:);
  if (~icv); oo = ones(nd,1); cxx = cx.*oo; cyy = cy.*oo; czz = cz.*oo; end
  line([cxx'; x'],[cyy'; y'],[czz'; z'],...
       'Parent',h,'Color',rc,'LineWidth',rw);
end

% plot frustum 
if (~isempty(frustum))

  if (icv); error('cannot specify both nonscalar cx, cy, cz and frustum'); end
  
  if (~isvector(frustum)||not(any(length(frustum)==[8,10])))
    error('frustum must be length 8 or 10');
  end
 
  fov = frustum(1:2);
  np = []; ip = 3;
  if (length(frustum)==10); np = frustum(3:4); ip = 5; end
  p = frustum(ip:ip+2); u = frustum(ip+3:ip+5);

  hf = frustumplot('fov',fov,'p',p,'u',u,'c',[cx, cy, cz],...
                   'uc',uc,'uw',uw,'da',da,'as',as,'aw',aw);
  set(hf,'Parent',h);

  % plot sample pixels
  if (dx)

    if (isempty(np)); error('frustum must be length 10'); end

    fp = 0.5*np./tan(0.5*fov); % focal lengths in pixels
    wp = 2*tan(0.5*fov)./np; % pixel widths
    xb = aim(p,u); % camera basis

    % transform data to camera frame
    cc = repmat([cx; cy; cz],1,prod(size(x)));
    xyzp = xb'*([x(:)'; y(:)'; z(:)'] - cc);
    xp = xyzp(1,:); yp = xyzp(2,:); zp = xyzp(3,:);

    % pixel upper left corners in pixels
    hh = mod(np,2)*0.5;
    u = floor(fp(1)*xp./zp+hh(1))-hh(1);
    v = floor(fp(2)*yp./zp+hh(2))-hh(2);

    % pixel boundary curves, one per column, in camera frame
    xp = wp(1)*[u; u+1; u+1; u; u];
    yp = wp(2)*[v; v; v+1; v+1; v];
    zp = ones(size(xp));

    % transform to world frame
    cc = repmat([cx; cy; cz],1,prod(size(xp)));
    xyzp = cc + xb*[xp(:)'; yp(:)'; zp(:)'];
   
    np = length(u);
    xx = mat2cell(reshape(xyzp(1,:), 5, np), 5, ones(1,np));
    yy = mat2cell(reshape(xyzp(2,:), 5, np), 5, ones(1,np));
    zz = mat2cell(reshape(xyzp(3,:), 5, np), 5, ones(1,np));

    drawcurves(h,1,'samplepix',xx,yy,zz,xc,xw);
  end
  
end

% if the axes was newly created, or if not but we replaced its old contents,
% then configure it appropriately
if (isempty(parent))
  if (~washold); hold('off'); end
  if (~ishold(a)); axescfg(a,ag,ar,av,kv,gr); end
end

if (fb); fitbbox(gca(),gr); end

if (zoom>0); camzoom(gca(),zoom); end

if (dbg); fprintf('sampleplot: %gs\n',toc(tstart)); end

end % sampleplot
