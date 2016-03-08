function [pct, sec, cp, cm] = patchcoverage(p, varargin)
% pct = patchcoverage(p) computes percent of good cells for boundary of patch p
%
%   The parameter p is a (scalar) Matlab struct describing the patch. See
%   the documentation for the function patchchk() for details.
%
%   The coverage evaluation is described in the paper "Sparse Surface Modeling
%   with Curved Patches" by Dimitrios Kanoulas and Marsette Vona, ICRA 2013.  A
%   grid of square cells is imposed on the local frame XY plane.  Cells are
%   categorized as good if (a) they contain a sufficient number of in-bounds
%   points relative to the portion of their area inside the patch boundary and
%   (b) they do not contain too many out-of-bounds points relative to their area
%   outside the patch boundary.
%
%   INPUTS
%
%   The neighborhood of points is taken from the first available of the
%   following sources (unless the source is explicitly given with the option
%   'src', see OPTIONS below):
%
%   * explicitly given with the options 'x', 'y', 'z'; if only 'x' and 'y' are
%   given then the data is in local frame, if all three coordinates are given it
%   is in world frame
%
%   * from the patch field p.fitbd, which for a fitted patch is the subset of
%   data points in local frame used for neighborhood fitting if the 'rbd' option
%   was specified in the call to patchfit()
%
%   * from the patch field p.fitsd, which for a fitted patch is the subset of
%   data points in world frame used for surface fitting if the 'rsd' option was
%   specified in the call to patchfit()
%
%   OUTPUTS
%
%   The output percentage, normalized to [0,1], is calculated as
%
%   pct = (ng-eo)/(nc-eo)
%
%   which is the ratio of good cells excluding exterior empty cells to the
%   total number cells excluding exterior empty cells.
%
%   Outputs sec, cp, and cm are optional.
%
%   Output sec is the runtime of the function call in seconds.
%
%   The cell parameter vector cp is
%
%   cp = [wc nc ng eo ba nd lc rc tc bc ti to zi zo ne]
%
%   wc - cell side length
%   nc - total number of cells
%   ng - number of good cells
%   eo - num empty cells outside boundary
%   ba - area of patch boundary
%   nd - total number of points in neighborhood
%   lc, rc, tc, bc - num cells left/right/above/below local frame origin
%   ti, to - conversion factors from cell in/out area to ic/oc thresholds
%   zi, zo - conversion factors from ne to ti, to
%   ne - expected number of points per in-bounds cell
%
%   The cell matrices are all (tc+bc)x(lc+rc).  The cell matrix cell array cm is
%
%   cm = {gd, ai, ic, oc}
%
%   gd - whether the cell is good
%   ai - intersection area of cell with bounds, normalized to [0,1]
%   ic, oc - num in-bounds/out-of-bounds points in each cell
%
%   OPTIONS
%
%   A list of (name,value) pairs may be given to set options.  Unrecognized 
%   names cause warnings.  If a name is given more than once the last-given 
%   (rightmost) value takes precedence.
%
%   Options 'x', 'y', 'z' (default []): coordinates of neighborhood points.
%   Either specify both 'x' and 'y', for data in local frame or all three for
%   data in world frame.
%
%   Option 'src' (default 'auto'): One of 'auto', 'xyz', 'fitbd', 'fitsd'.
%   Explicitly requests that the neighborhood data be taken from the indicated
%   source.
%   
%   Option 'wc' (default 0.01): sets the cell side length if positive.
%
%   Option 'nc' (default 0): sets the desired number of cells if positive.  Note
%   that the actual number of cells may be greater because local frame origin
%   must fall on cell boundaries.  Exactly one of wc, nc must be positive.
%
%   Options 'ti', 'to' (default 0): explicit conversion factors from cell
%   in/out area to ic/oc thresholds.
%
%   Options 'zi', 'zo' (default 0.5, 0.5): conversion factors from ne to ti and
%   to.  Exactly one of ti, zi must be positive, same for to, zo.
%
%   Option 'ne' (default 0): if positive overrides number of expected points
%   per in-bounds cell.  Otherwise ne = nd*wc*wc/ba.
%
%   Option 'dbg' (default 0): may have any nonnegative value. dbg=0 disables
%   debug.  Larger values enable successively more verbose debug.
%
% Copyright (C) 2013 Marsette Vona and Dimitrios Kanoulas

tstart = tic(); % timing

% process (name,value) options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
ix = []; iy = []; iz = []; src = 'auto';
wc = 0.01; nc = 0; inc = 0;
ti = 0; to = 0; zi = 0.5; zo = 0.5; ne = 0; ine = 0;
iti = 0; ito = 0;
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'x'; ix = v; case 'y'; iy = v; case 'z'; iz = v;
      case 'src'; src = v;
      case 'wc'; wc = v; case 'nc'; nc = v; inc = 1;
      case 'ti'; ti = v; iti = 1; case 'to'; to = v; ito = 1;
      case 'zi'; zi = v; case 'zo'; zo = v; case 'ne'; ne = v; ine = 1;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

if ((wc<=0)&&(nc<=0))
  error('one of wc, nc must be positive');
end

if (((ti<=0)&&(zi<=0))||((ti>0)&&(zi>0)))
  error('exactly one of ti, zi must be positive');
end

if (((to<=0)&&(zo<=0))||((to>0)&&(zo>0)))
  error('exactly one of to, zo must be positive');
end

if (ine&&(ne<0)); error('ne must be nonnegative'); end

% get data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

iw = 0; % input in world frame?

x = []; y = []; z = [];
isauto = strcmpi(src,'auto');

if ((isauto||strcmpi(src,'xyz'))&&~isempty(ix)&&~isempty(iy))
  x = ix; y = iy; z = iz; iw = ~isempty(iz);
end

if (((isauto&&isempty(x))||strcmpi(src,'fitbd'))&&isfield(p,'fitbd'))
  x = p.fitbd(:,1); y = p.fitbd(:,2); z = p.fitbd(:,3); iw = 0;
end

if (((isauto&&isempty(x))||strcmpi(src,'fitsd'))&&isfield(p,'fitsd'))
  x = p.fitsd(:,1); y = p.fitsd(:,2); z = p.fitsd(:,3); iw = 1;
end

%if (isempty(x)); error('no data'); end

sz = size(x); nd = sz(1)*sz(2);

if (any(size(y)~=sz)||(iw&&any(size(z)~=sz)))
  error('data must all be same size');
end

x = x(:); y = y(:); % vectorize

if (iw) % transform to local frame
  z = z(:);
  rrinv = rexp(-p.r);
  xyz = [x,y,z]*rrinv'+repmat((-rrinv*p.c(:))',nd,1);
  x = xyz(:,1); y = xyz(:,2);
end

% determine parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = patchchk(p,'gb',1); % update boundary fields

% patch bounding box and boundary area
bb = p.bb; ba = p.ba;
xmin = bb(1,1); xmax = bb(1,2); ymin = bb(2,1); ymax = bb(2,2);

% expand bounding box to fit all data
if (nd>0)
  xmin = min(xmin,min(x)); xmax = max(xmax,max(x));
  ymin = min(ymin,min(y)); ymax = max(ymax,max(y));
end

assert((xmin<=0)&&(0<=xmax));
assert((ymin<=0)&&(0<=ymax));

% infer wc from nc
if (inc&&(nc>0)); wc = sqrt((xmax-xmin)*(ymax-ymin)/nc); end

assert(wc>0);

% cell grid left, right, top, bottom extents
lc = abs(floor(xmin/wc)); rc = ceil(xmax/wc);
bc = abs(floor(ymin/wc)); tc = ceil(ymax/wc);

cr = tc+bc; cc = lc+rc; % num cell rows, cols
nc = cr*cc; % actual number of cells

% infer expected number of points per in-bounds cell
if (~ine); ne = nd*wc*wc/ba; end

if (~iti); ti = zi*ne; end
if (~ito); to = zo*ne; end

if (dbg)
  fprintf('wc=%g, nc=%g, ti=%g, to=%g, zi=%g, zo=%g, ne=%g=%g/%g\n',...
          wc,nc,ti,to,zi,zo,ne,nd,ba/(wc*wc));
end

assert(ne>=0);
assert(ti>=0);
assert(to>=0);

% create cell matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% cell at coords (i,j)=(0,0) has its lower left corner at local frame origin
ccx = -lc:(rc-1); ccy = (tc-1):-1:-bc; [cx,cy] = meshgrid(ccx,ccy);

ii = p.bl(x,y)<=0; % bool index of in-bounds points

ni = sum(ii); no = nd-ni;

ctrs = {wc/2+ccx(:)*wc, wc/2+flipud(ccy(:))*wc}; % centers of grid cells

if (ni>0)
  ic = hist3([x(ii), y(ii)],'Ctrs',ctrs); % bin interior datapoints
else ic = zeros(cc, cr); end

if (no>0)
  oc = hist3([x(~ii),y(~ii)],'Ctrs',ctrs); % bin exterior datapoints
else oc = zeros(cc, cr); end

% dbg
%hist3([x(ii), y(ii)], 'Ctrs',ctrs,'FaceColor','b','FaceAlpha',0.5); 
%hist3([x(~ii),y(~ii)],'Ctrs',ctrs,'FaceColor','b','FaceAlpha',0.5);
%fitbbox(gca());

ic = rot90(ic); oc = rot90(oc); % wtf

ai = min(p.bc(cx,cy,wc)/(wc*wc),1); ao = 1-ai; % normalized area of intersection

% bool index of good cells
igd = ic>=round(ai*ti); ogd = oc<=round(ao*to); gd = igd&ogd;

% generate outputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ng = sum(gd(:));

%eo = sum(sum((ao.*((ic+oc)==0))); % num empty outside cells
eo = sum(sum((ao==1)&((ic+oc)==0))); % num empty outside cells

num = ng-eo; den = nc-eo;

assert(den>=0); % because nc>=eo
assert(num<=den); % because ng<=nc

if (num<0); warning('(ng=%g)<(eo=%g)',ng,eo); num = 0; end

pct = num/den;

if (dbg); fprintf('pct=%g\n',pct); end

t = toc(tstart); if (nargout>1); sec = t; end
if (nargout>2); cp = [wc nc ng eo ba nd lc rc tc bc ti to zi zo ne]; end
if (nargout>3); cm = {gd, ai, ic, oc}; end

end
