function h = frustumplot(varargin)
% frustumplot(...) plots a frustum in the current axes
%
%   All arguments must be given as name,value pairs.  See OPTIONS below.
%   Unrecognized names cause warnings.  If a name is given more than once the
%   last-given (rightmost) value takes precedence.
%
%   The plot is made in the current axes of the current figure, if any,
%   else a new figure window is opened.  The return value h is the handle
%   of a new Matlab hgtransform group containing the generated graphics.
%
%   If called with hold off then the plot replaces the current contents of the
%   current axes, if any, and the axes are reconfigured for the plot. If called
%   with hold on the plot is added to the current axes, which are not
%   reconfigured.
%
%   OPTIONS
%
%   Option 'fov' (default [58,45]*(pi/180)): field of view in radians.  If
%   scalar then the frustum aspect ratio is square with equal horizontal and
%   vertical fields of view.  If 1x2 or 2x1 then fov(1) is the horizontal and
%   fov(2) the vertical fov.  The default is the Kinect fovs.
%
%   Options 'p', 'u', 'c', 'ydir' (default resp. [0,0,1]', [0,-1,0]', [0,0,0]',
%   'ydown'): pointing vector, up vector, center point, and y direction of
%   camera frame.  Mutually exclusive with 'xform'.  The transform from camera
%   to parent frame is calculated as [aim(p,u,ydir),c(:);0,0,0,1].
%
%   Option 'xform' (default []): transform from camera to parent frame.
%   Overrides 'p', 'u', 'c', 'ydir' if nonempty.  Must be either, 3x4, 4x4, or
%   3x2. If four columns are given, the first three rows are taken as the first
%   three rows of a homogenous rigid body transformation matrix (the rigid body
%   constraints are not checked).  If two columns are given, the second gives a
%   translation vector and the first a rotation vector for rexp().
%
%   Option 'du' (default 1): whether to draw the frustum
%
%   Option 'ud' (default 1): frustum depth
%
%   Option 'uc' (default 'm'): frustum line color, either a scalar color name
%   or [r g b].
%
%   Option 'uw' (default 0.5): frustum line width in points
%
%   Option 'da' (default 1): whether to draw RGB axes triad for the frustum.
%
%   Option 'aw' (default 0.5): axes triad line width in points
%
%   Option 'as' (default 1): local frame axes triad scale
%
%   Options 'ag', 'ar': see axescfg(), applies when 'parent' is nonempty
%
%   Option 'parent' (default []): if nonempty then this is the parent for
%   the generated hgtransform
%
%   Option 'refresh' (default []): if nonempty then must be handle to an
%   hgtransform of a previously plotted volume.  The graphics are refreshed.
%
%   Option 'dbg' (default 0): may have any nonnegative value. dbg=0
%   disables debug. Larger values enable successively more verbose debug.
%
% Copyright (C) 2013 Marsette A. Vona

tstart = tic();

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expected even number of varargs (name,value pairs), got %d',nva);
  nva = nva-1;
end
nopt = nva/2;

% option defaults
fov = [58,45]*(pi/180);
p = [0,0,1]'; u = [0,-1,0]'; c = [0,0,0]'; ydir = 'ydown'; xform = [];
du = 1; ud = 1; uw = 0.5; uc = 'm';
da = 1; aw = 0.5; as = 1;
ar = 1; ag = 0;
parent = []; refresh = [];
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'fov'; fov = v; case 'p'; p = v; case 'u'; u = v; case 'c'; c = v;
      case 'ydir'; ydir = v; case 'xform'; xform = v;
      case 'du'; du = v; case 'ud'; ud = v;
      case 'uw'; uw = v; case 'uc'; uc = v;
      case 'da'; da = v; case 'aw'; aw = v; case 'as'; as = v;
      case 'ar'; ar = v; case 'ag'; ag = v;
      case 'parent'; parent = v; case 'refresh'; refresh = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

% parse fov
if (isscalar(fov)); hfov = fov; vfov = fov;
else hfov = fov(1); vfov = fov(2); end

% parse xform
if (isempty(xform)); xm = [aim(p,u,ydir),c(:);0,0,0,1];
else
  [nr, nc] = size(xform);
  if ((nc==4)&&((nr==4)||(nr==3))); xm = xform;
  elseif ((nc==2)&&(nr==3)); xm = [rexp(xform(:,1)),xform(:,2)];
  else error('xform must be 3x4, 4x4, or 3x2'); end
  if (nr==3); xm = [xm; 0 0 0 1]; end
end

% get axes
if (isempty(refresh)&&isempty(parent))
  a = newplot(); % gets current or makes new, as appropriate
  washold = ishold(); hold('on');
end

% create transform group
if (~isempty(refresh)); h = refresh(1); else h = hgtransform(); end
set(h,'Matrix',xm);
set(h,'Tag','frustumplot');
if (~isempty(parent)); set(h,'Parent',parent); end

% draw boundary
lx = -ud*tan(hfov/2); ly = -ud*tan(vfov/2); ux = -lx; uy = -ly; uz = ud;
bx = {[lx ux ux lx lx], [0 lx], [0 ux], [0 ux], [0 lx]};
by = {[ly ly uy uy ly], [0 ly], [0 ly], [0 uy], [0 uy]};
bz = {[uz uz uz uz uz], [0 uz], [0 uz], [0 uz], [0 uz]};
drawcurves(h,du,'boundary',bx,by,bz,uc,uw);

drawtriad(h,da,as,'d',ud,aw); % camera frame axes triad

% if the axes was newly created, or if not but we replaced its old contents,
% then configure it appropriately
if (isempty(refresh)&&isempty(parent))
  if (~washold); hold('off'); end
  if (~ishold(a)); axescfg(a,ag,ar); end
end

if (dbg); fprintf('frustumplot: %gs\n',toc(tstart)); end

end % frustumplot
