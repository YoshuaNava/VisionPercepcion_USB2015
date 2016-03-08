function [mesh, area] = samplemesh(varargin)
% mesh = samplemesh(...) creates or modifies a grid-organized triangle mesh
%
%   Output mesh is in one of the formats described below controlled by the
%   'otype' argument.
%
%   Second output area is optional, if requested it is the sum 3D area of the
%   triangles in the mesh.  xyz vertex coordinates must be provided.
%
%   All arguments must be given as name,value pairs. Unrecognized names cause
%   warnings.  If a name is given more than once the last-given (rightmost)
%   value takes precedence.
%
%   At least one of the argument sets {'x','y','z'}, 'z', 'adjtri', or 'sz' must
%   be given.  These establish the vertex array size MxN.  Both M and N must be
%   greater than 1.  If more than one of these input sets are given then they
%   must agree in size, and valid triangles result from the intersection of the
%   given inputs.
%
%   Arguments 'x', 'y', 'z' give the vertex coordinates, or nan for invalid
%   vertices.
%
%   Argument 'adjtri' specifies the triangles adjacent to each vertex in the
%   format described below.
%
%   Argument 'sz' is either 1x2 or 2x1 and gives M and N directly.
%
%   MESH FORMATS
%
%   'adjtri': an MxN integer array where the first six bits of each entry 
%   specify the presence of up to 6 triangles adjacent to the corresponding
%   vertex v, like this
%
%      *--*
%      |\3|\
%      |4\|2\
%      *--v--*
%       \5|\1|
%        \|0\|
%         *--*
%
%   'faces': a (2*(M-1)*(N-1))x3 matrix of linear indices into the vectorized
%   form of the MxN vertex coordinate arrays.  Every vertex v except those in
%   the last row and column has two triangles l and u associated to it, like
%   this
%
%      v--*
%      |\u|
%      |l\|
%      *--*
%
%   and each row of the faces matrix specifies the vertex indices for a triangle
%   in CCW order.  Suppressed triangles are indicated by a row of nan.  All
%   the lower faces are given first organized column major, then all the
%   upper faces, also organized column major.
%
%   'cleanfaces': like 'faces' except rows for suppressed triangles are
%   deleted entirely, which breaks grid organization.
%
%   OPTIONS
%
%   A list of name,value pairs may be given to set options. Unrecognized
%   names cause warnings. If a name is given more than once the last-given
%   (rightmost) value takes precedence.
%
%   Option 'edgedetect' (default 0): either a scalar integer or a cell array to
%   give options for the matlab edge() image processing function to detect depth
%   discontinuities in the vertex coordinates.  If 0 or negative then the
%   function is not run; positive integers use option presets.  The edge()
%   function is run on the 'z' vertex coordinate data, which must also be given;
%   resulting edge pixels are treated as invalid.
%
%   Option 'markbad' (default 1): whether to suppress triangles with invalid
%   (nan or edge) vertices
%
%   Option 'maxedge' (default []): if nonempty then suppress triangles which
%   have an edge longer than this; requires xyz vertex coordinate data to
%   also be given
%
%   Option 'maxedgeratio' (default 5): if nonempty then suppress trianges
%   which have a ratio of maxsidelength/minsidelength greater than this;
%   requires xyz vertex coordinate data to also be given
%
%   Option 'otype' (default 'adjtri'): output mesh format, one of 'adjtri',
%   'faces', 'cleanfaces', see descriptions above.  
%
%   Options 'decimate', 'lowres': see samplecvt()
%
%   Option 'dbg' (default 0): may have any nonnegative value. dbg=0 disables
%   debug.  Larger values enable successively more verbose debug.
%
% Copyright (C) 2013 Marsette A. Vona

tstart = tic();

% option presets %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% default edge detect options
dedo1 = {'canny'};
dedo2 = {'canny',0.2};
dedo = {dedo1, dedo2};

% argument handling %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
x = []; y = []; z = []; ixyz = 0; iz = 0;
adjtri = []; iadj = 0; sz = []; isz = 0;
edgedetect = 0;
markbad = 1;
maxedge = []; maxedgeratio = 5;
otype = 'adjtri'; 
decimate = 0; lowres = 0;
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'x'; x = v; case 'y'; y = v; case 'z'; z = v;
      case 'adjtri'; adjtri = v;
      case 'sz'; sz = v;
      case 'edgedetect'; edgedetect = v;
      case 'markbad'; markbad = v;
      case 'maxedge'; maxedge = v; case 'maxedgeratio'; maxedgeratio = v;
      case 'otype'; otype = v;
      case 'decimate'; decimate = v; case 'lowres'; lowres = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

% apply decimate and lowres %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

iz = ~isempty(z); ixyz = ~isempty(x)&&~isempty(y)&&iz;
iadj = ~isempty(adjtri); isz = ~isempty(sz);

if (decimate||lowres)
  if (~ixyz); error('decimate and lowres require xyz'); end
  [x y z] = samplecvt('x',x,'y',y,'z',z,'decimate',decimate,'lowres',lowres);
end

% check input constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (~(ixyz||iz||iadj||isz))
  error('must give either {x, y, z}, z, adjtri, or sz');
end

% take sz from sz input, if given, else from z or adjtri
if (~isz)
  if (iz); sz = size(z);
  elseif (iadj); sz = size(adjtri); end
end

assert(~isempty(sz));

nr = sz(1); nc = sz(2);

if ((nr<=1)||(nc<=1)); error('vertex array dims must be greater than 1'); end

if ((iz&&~isequal(sz,size(z)))||...
    (ixyz&&(~isequal(sz,size(x))||~isequal(sz,size(y))))||...
    (iadj&&(~isequal(sz,size(adjtri)))))
  error('supplied data must all be same size');
end

edo = {}; % edge() options to use, or empty to not run edge()
if (~isempty(edgedetect))
  if (~iz); error('edgedetect requires z input data'); end
  if (iscell(edgedetect)); edo = edgedetect;
  elseif (isscalar(edgedetect)&&(edgedetect>0)&&(edgedetect<=length(dedo)))
    edo = dedo(edgedetect); edo = edo{:};
  elseif (~isscalar(edgedetect)||(edgedetect>0)) 
    error('edgedetect must be cell array or int in [0,%d]',length(dedo));
  end
end

if ((~isempty(maxedge)||~isempty(maxedgeratio)||(nargout>1))&&~ixyz)
  error('area, maxedge, maxedgeratio require xyz');
end

if ((~isempty(maxedge)&&~isscalar(maxedge))||...
    (~isempty(maxedgeratio)&&~isscalar(maxedgeratio)))
  error('maxedge and maxedgeratio must be empty or scalar');
end

% start with all triangles good %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% v--*
% |\u|
% |l\|
% *--*

lok = true(nr-1,nc-1);
uok = true(nr-1,nc-1);

% mask from given adjacency matrix %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (iadj)

  % *--*     *--*   
  % |\3|\    |\u|\  
  % |4\|2\   |l\|l\ 
  % *--v--*  *--v--*
  %  \5|\1|   \u|\u|
  %   \|0\|    \|l\|
  %    *--*     *--*

  mm = bitand(adjtri,2^0); lok = lok&mm(1:nr-1,1:nc-1);
  mm = bitand(adjtri,2^1); uok = uok&mm(1:nr-1,1:nc-1);

  mm = bitand(adjtri,2^2); lok = lok&mm(2:nr,1:nc-1);

  mm = bitand(adjtri,2^3); uok = uok&mm(2:nr,2:nc);
  mm = bitand(adjtri,2^4); lok = lok&mm(2:nr,2:nc);

  mm = bitand(adjtri,2^5); uok = uok&mm(1:nr-1,2:nc);

end

% mask bad triangles from bad verts %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (markbad)

  gv = true(nr,nc); % good verts

  % a vert with any nan coord is bad
  if (iz); gv = ~isnan(z); end
  if (ixyz); gv = gv&(~isnan(x))&(~isnan(y)); end

  % edge verts are bad
  if (~isempty(edo)); gv = gv&(~edge(z,edo{:})); end

  % each bad vert makes up to 6 bad tris

  % *--*     *--*   
  % |\3|\    |\u|\  
  % |4\|2\   |l\|l\ 
  % *--v--*  *--v--*
  %  \5|\1|   \u|\u|
  %   \|0\|    \|l\|
  %    *--*     *--*

  lok = lok&gv(1:nr-1,1:nc-1); %0
  uok = uok&gv(1:nr-1,1:nc-1); %1

  lok = lok&gv(2:nr,1:nc-1);   %2

  uok = uok&gv(2:nr,2:nc);     %3
  lok = lok&gv(2:nr,2:nc);     %4

  uok = uok&gv(1:nr-1,2:nc);   %5

end

% generate edge lengths %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (~isempty(maxedge)||~isempty(maxedgeratio)||(nargout>1))

  % v-he-*
  % |\ U |
  % v d  |
  % e  e |
  % | L \|
  % *----*

  hel = sqrt((x(:,2:nc)-x(:,1:nc-1)).^2+...
             (y(:,2:nc)-y(:,1:nc-1)).^2+...
             (z(:,2:nc)-z(:,1:nc-1)).^2);

  vel = sqrt((x(2:nr,:)-x(1:nr-1,:)).^2+...
             (y(2:nr,:)-y(1:nr-1,:)).^2+...
             (z(2:nr,:)-z(1:nr-1,:)).^2);

  del = sqrt((x(2:nr,2:nc)-x(1:nr-1,1:nc-1)).^2+...
             (y(2:nr,2:nc)-y(1:nr-1,1:nc-1)).^2+...
             (z(2:nr,2:nc)-z(1:nr-1,1:nc-1)).^2);
end

% mask bad triangles based on edges %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (~isempty(maxedge))

  lok = lok&(del<=maxedge);
  uok = uok&(del<=maxedge);

  lok = lok&(vel(1:nr-1,1:nc-1)<=maxedge);
  uok = uok&(hel(1:nr-1,1:nc-1)<=maxedge);

  lok = lok&(hel(2:nr,1:nc-1)<=maxedge);
  uok = uok&(vel(1:nr-1,2:nc)<=maxedge);

end

if (~isempty(maxedgeratio))

  doverh = del           ./hel(1:nr-1,:); doverh = doverh(:);
  doverv = del           ./vel(:,1:nc-1); doverv = doverv(:);

  hovervu = hel(1:nr-1,:)./vel(:,2:nc);   hovervu = hovervu(:);
  dovervu = del          ./vel(:,2:nc);   dovervu = dovervu(:);

  voverhl = vel(:,1:nc-1)./hel(2:nr,:);   voverhl = voverhl(:);
  doverhl = del          ./hel(2:nr,:);   doverhl = doverhl(:);

  muer = max([doverh,1./doverh,hovervu,1./hovervu,dovervu,1./dovervu],[],2);
  uok = uok&(reshape(muer,nr-1,nc-1)<=maxedgeratio);

  mler = max([doverv,1./doverv,voverhl,1./voverhl,doverhl,1./doverhl],[],2);
  lok = lok&(reshape(mler,nr-1,nc-1)<=maxedgeratio);

end

% generate mesh %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (strcmpi(otype,'adjtri'))

  % *--*     *--*   
  % |\3|\    |\u|\  
  % |4\|2\   |l\|l\ 
  % *--v--*  *--v--*
  %  \5|\1|   \u|\u|
  %   \|0\|    \|l\|
  %    *--*     *--*

  adjtri = uint8(zeros(nr,nc)); fc = false(nr-1,1); fr = false(1,nc-1);

  adjtri = bitor(adjtri,uint8(2^0*[lok,fc;  fr,false]));
  adjtri = bitor(adjtri,uint8(2^1*[uok,fc;  fr,false]));

  adjtri = bitor(adjtri,uint8(2^2*[false,fr;lok,fc]));

  adjtri = bitor(adjtri,uint8(2^3*[false,fr;fc,uok]));
  adjtri = bitor(adjtri,uint8(2^4*[false,fr;fc,lok]));

  adjtri = bitor(adjtri,uint8(2^5*[fc,uok;  false,fr]));

  mesh = adjtri;

elseif (strcmpi(otype,'faces')||strcmpi(otype,'cleanfaces'))

  % v--c
  % |\u|
  % |l\|
  % a--b

  ff = reshape(1:nr*nc,nr,nc);  % MxN with linear index in each position
  ff = ff(1:nr-1,1:nc-1);       % cut off last row and col
  ff = ff(:);                   % vectorize to (F/2)x1, F=2*(M-1)*(N-1)
  vi = ff; ai = ff+1; bi = ff+nr+1; ci = ff+nr;
  lf = [vi,ai,bi]; % lower faces vab (F/2)x3
  uf = [vi,bi,ci]; % upper faces vbc (F/2)x3 (upper)

  if (strcmpi(otype,'cleanfaces')); lf = lf(lok(:),:); uf = uf(uok(:),:);
  else lf(~lok(:),:) = nan; uf(~uok(:),:) = nan; end

  mesh = [lf; uf];

else error('unrecognized otype %s',otype); end

% generate mesh area %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (nargout>1)

  % v-he-*
  % |\ U |
  % v d  |
  % e  e |
  % | L \|
  % *----*

  a = vel(:,1:nc-1); b = hel(2:nr,:); c = del; s = 0.5*(a+b+c);
  la = sqrt(s.*(s-a).*(s-b).*(s-c)); % Heron's formula

  a = vel(:,2:nc); b = hel(1:nr-1,:); c = del; s = 0.5*(a+b+c);
  ua = sqrt(s.*(s-a).*(s-b).*(s-c));

  area = sum(la(lok))+sum(ua(uok));

end

if (dbg); fprintf('samplemesh: %gs\n',toc(tstart)); end

end
