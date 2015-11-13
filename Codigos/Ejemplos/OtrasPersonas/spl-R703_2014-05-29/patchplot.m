function h = patchplot(p, varargin)
%h = patchplot(p) plots patch p
%
%   The parameter p is a (scalar) Matlab struct describing the patch. See
%   the documentation for the function patchchk() for details.
%
%   The plot is made in the current axes of the current figure, if any,
%   else a new figure window is opened.  The return value h is the handle
%   of a new Matlab hgtransform group containing the generated graphics.
%
%   If called with hold off then the patch plot replaces the current
%   contents of the current axes, if any, and the axes are reconfigured for
%   the patch. If called with hold on, the patch is added to the current
%   axes, which are not reconfigured.
%
%   OPTIONS
%
%   A list of name,value pairs may be given to set options. Unrecognized
%   names cause warnings. If a name is given more than once the last-given
%   (rightmost) value takes precedence.
%
%   Options 'ss','gd': see documentation in patchchk()
%
%   Option 'da' (default 1): whether to draw RGB local frame axes triad for
%   the patch.
%
%   Option 'db' (default 1): whether to draw patch boundary curve.
%
%   Option 'dg' (default 1): whether to draw patch grid.
%
%   Option 'df' (default 1): whether to draw patch faces.
%
%   Option 'dc' (default []): type of patch coverage grid to draw.  Requres
%   fields 'cvgp' and 'cvgm' to be set on the patch corresponding to the cp and
%   cm outputs of patchoverage().  Either empty to disable or one of 'g',
%   'i', 'o', 'a' to color cells by good flag, number of inside points,
%   nubmer of outside points, or intersection area.  If a 2x1 or 1x2 cell array
%   then the first element is one of those characters and the second is the z
%   offset of the grid graphics in local frame.
%
%   Option 'fc' (default 'w'): face color, either a color name, [r g b], or
%   'none'
%
%   Option 'fa' (default 1): face alpha, a scalar in the interval [0,1]
%
%   Option 'ec' (default 'none'): face edge color, either a color name,
%   [r g b], or 'none'
%
%   Option 'bc' (default 'k'): boundary color, either a color name or
%   [r g b]
%
%   Option 'bw' (default 0.5): boundary line width in points
%
%   Option 'gc' (default 'k'): patch grid color, either a color name or
%   [r g b]
%
%   Option 'gw' (default 0.5): patch grid line width in points
%
%   Option 'aw' (default 0.5): local frame axes triad line width in points
%
%   Option 'as' (default ss): local frame axes triad scale, or 'ss' to use
%   p.ss
%
%   Options 'ag', 'ar', 'av', 'kv', 'gr': see axescfg(), applies when
%   'refresh' and 'parent' are both nonempty
%
%   Option 'fb' (default 0): whether to fit axes limits to all axes contents
%   (not just the new contents) with fitbbox()
%
%   Option 'parent' (default []): if nonempty then this is the parent for
%   the generated hgtransform
%
%   Option 'refresh' (default []): if nonempty then must be handle to an
%   hgtransform of a previously plotted patch.  The graphics are refreshed.
%
% Copyright (C) 2013 Marsette A. Vona

tstart = tic();

% coverage grid rendering parameters
cec = 'k'; cea = 0.5; cel = 'flat'; clw = 0.5;
cfa = 0.5; cfl = 'flat';
cfcmin = [1 0 0]; cfcmax = [0 1 0];

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
ss = 1; oss = 0; gd = 0; ogd = 0;
da = 1; db = 1; dg = 1; df = 1; dc = [];
fc = 'w'; fa = 1; ec = 'none';
bc = 'k'; bw = 0.5; gc = 'k'; gw = 0.5;
aw = 0.5; as = 'ss';
ag = 0; ar = 1; av = 0; kv = 0; gr = 1; fb = 0;
dbg = 0;
parent = []; refresh = [];

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'ss'; ss = v; oss = 1; case 'gd'; gd = v; ogd = 1;
      case 'da'; da = v; case 'db'; db = v; case 'dg'; dg = v;
      case 'df'; df = v; case 'dc'; dc = v;
      case 'fc'; fc = v; case 'fa'; fa = v; case 'ec'; ec = v;
      case 'bc'; bc = v; case 'bw'; bw = v;
      case 'gc'; gc = v; case 'gw'; gw = v;
      case 'aw'; aw = v; case 'as'; as = v;
      case 'ag'; ag = v; case 'ar'; ar = v; case 'av'; av = v;
      case 'kv'; kv = v; case 'gr'; gr = v; case 'fb'; fb = v;
      case 'dbg'; dbg = v;
      case 'parent'; parent = v; case 'refresh'; refresh = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name');
  end
end

% Call patchchk to generate grid and boundary samples and face triangulation.
% Non strict to avoid warnings when the curvature values are incorrect for the
% patch type, which can happen during patchfit debug.
args = {p,'gp',1,'gs',(dg||db),'gt',df,'strict',0};
if (oss); args = {args{:},'ss',ss}; end
if (ogd); args = {args{:},'gd',gd}; end
p = patchchk(args{:});

% get axes
if (isempty(refresh)&&isempty(parent))
  a = newplot(); % gets current or makes new, as appropriate
  washold = ishold(); hold('on');
end

% create transform group
if (~isempty(refresh)); h = refresh(1); else h = hgtransform(); end
set(h,'Matrix',p.pm);
set(h,'Tag','patchplot');
if (~isempty(parent)); set(h,'Parent',parent); end

% draw faces
o = findobj(get(h,'Children'),'flat','Tag','faces');
if (df)
  if (isempty(o)); o = patch('Parent',h,'Tag','faces'); end
  set(o,'Faces',p.ft,'Vertices',p.fv, ...
        'FaceColor',fc,'FaceAlpha',fa,'EdgeColor',ec);
else delete(o);
end

% draw grid (if not then don't require p.gv)
o = findobj(get(h,'Children'),'flat','Tag','gridlines');
if (dg) drawcurves(h,1,'gridlines',p.gv{1},p.gv{2},p.gv{3},gc,gw);
else delete(o);
end

% draw boundary
o = findobj(get(h,'Children'),'flat','Tag','boundary');
if (db)
  if (isempty(o)); o = line('Parent',h,'Tag','boundary'); end
  set(o,'XData',p.bv(:,1),'YData',p.bv(:,2),'ZData',p.bv(:,3),...
        'Color',bc,'LineWidth',bw);
else delete(o);
end

% draw coverage grid
o = findobj(get(h,'Children'),'flat','Tag','coverage');
if (~isempty(dc)&&...
    isfield(p,'cvgp')&&~isempty(p.cvgp)&&isfield(p,'cvgm')&&~isempty(p.cvgm))

  wc = p.cvgp(1);
  lc = p.cvgp(7); rc = p.cvgp(8); tc = p.cvgp(9); bc = p.cvgp(10);
  cr = tc+bc; cc = lc+rc;
  
  if (iscell(dc)); tt = dc{1}; zo = dc{2}; else tt = dc; zo = 0; end
  
  switch (tt)
    case 'g'; cg = p.cvgm{1};
    case 'i'; cg = p.cvgm{3}; cg = cg/max(max(cg));
    case 'o'; cg = p.cvgm{4}; cg = cg/max(max(cg));
    case 'a'; cg = p.cvgm{2};
    otherwise; warning('unrecognized coverage grid type %s',tt);
  end
  
  % grid vertices
  ccx = -lc:rc; ccx = ccx*wc; ccy = tc:-1:-bc; ccy = ccy*wc;
  [cx,cy] = meshgrid(ccx,ccy); cz = ones(size(cx))*zo;
  vv = [cx(:) cy(:) cz(:)];
  
  % v--c
  % |  |
  % |  |
  % a--b
  
  % grid faces
  nr = cr+1; nc = cc+1;
  ff = reshape(1:nr*nc,nr,nc);  % NRxNC linear index in each pos
  ff = ff(1:cr,1:cc);           % cut off last row and col
  ff = ff(:);                   % vectorize to NFx1, NF=CR*CC
  assert(length(ff)==(cr*cc));
  vi = ff; ai = ff+1; bi = ff+nr+1; ci = ff+nr;
  ff = [vi ai bi ci];
  
  % face colors
  cg = cg(:);
  assert(length(cg)==length(ff));
  cg = repmat(cg,1,3);
  fc = (1-cg)*diag(cfcmin)+cg*diag(cfcmax);
  
  if (isempty(o)); o = patch('Parent',h,'Tag','coverage'); end
  set(o,'Vertices',vv,'Faces',ff,...
      'EdgeColor',cec,'EdgeAlpha',cea,'EdgeLighting',cel,'LineWidth',clw,...
      'FaceAlpha',cfa,'FaceLighting',cfl,...
      'FaceColor','flat','FaceVertexCData',fc,...
      'SpecularStrength',0.2,'DiffuseStrength',0.5);
  
else delete(o);
end

drawtriad(h,da,as,'ss',p.ss,aw); % local frame axes triad

% if the axes was newly created, or if not but we replaced its old
% contents, then configure it appropriately
if (isempty(refresh)&&isempty(parent))
  if (~washold); hold('off'); end
  if (~ishold(a)); axescfg(a,ag,ar,av,kv,gr); end
end
  
if (fb) fitbbox(gca(),gr); end

if (dbg); fprintf('patchplot: %gs\n',toc(tstart)); end

end % patchplot

