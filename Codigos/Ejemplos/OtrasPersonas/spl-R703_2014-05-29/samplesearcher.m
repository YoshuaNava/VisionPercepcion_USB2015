function searcher = samplesearcher(x,y,z,k,varargin)
% searcher = samplesearcher(x,y,z,k,...) creates a neighborhood searcher
%
%   Arguments x, y, z give the sample coordinates and must all be the same size
%   MxN.  For breadth-first search (see METHODS below) the data matrices must be
%   grid organized.
%
%   Argument k is the maximum number of neighbors to return for each search.
%
%   The return is a function nbrs = search(seed,radius) that can be used to
%   repetitively search for spatial neighborhoods in the data.  Argument seed is
%   a scalar linearized index into the xyz sample data and argument radius is
%   the neighborhood radius to search.  Return nbrs is a vector of up to k
%   linear indices of nearest neighbors of seed within distance radius.
%
%   METHODS
%
%   'kdtree' and 'exhaustive' use the Matlab KDTreeSearcher and
%   ExhaustiveSearcher classes, see the documentation for createns() and
%   rangesearch().  If rangesearch() returns more than k neighbors then a random
%   subset of k of them is taken.
%
%   'bfsdd' and 'bfsmd' perform breadth-first searches along edges of the
%   triangle mesh described by option 'adjtri', which must also be given.
%   'bfsdd' uses the direct spatial distance from the seed; 'bfsmd' uses the
%   geodesic distance along BFS edges from seed.
%
%   OPTIONS
%
%   A list of name,value pairs may be given to set options. Unrecognized
%   names cause warnings. If a name is given more than once the last-given
%   (rightmost) value takes precedence.
%
%   Option 'method' (default 'kdtree'): one of 'kdtree', 'exhaustive',
%   'bfsdd', 'bfsmd'; see METHODS above.
%
%   Option 'adjtri' (default []): organized triangle mesh in the adjacency
%   format described in samplemesh.m.  adjtri is required for any of the BFS
%   methods and its dimensions must match the xyz data.
%
%   Option 'dbg' (default 0): may have any nonnegative value. dbg=0 disables
%   debug.  Larger values enable successively more verbose debug.
%
% Copyright (C) 2013 Marsette A. Vona

tstart = tic();

% argument handling %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
method = 'kdtree'; adjtri = [];
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'method'; method = v; case 'adjtri'; adjtri = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

% check input constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sz = size(x); nr = sz(1); nc = sz(2);

if (~isequal(sz,size(y))||~isequal(sz,size(z))||...
    (~isempty(adjtri)&&~isequal(sz,size(adjtri))))
  error('supplied data must all be same size');
end

if (~isscalar(k)||(k<1)); error('k must be scalar greater than 1'); end

if (strncmpi('bfs',method,3)&&isempty(adjtri))
  error('bfs methods require adjtri');
end

% kdtree and exhaustive use Matlab NeighborSearcher %%%%%%%%%%%%%%%%%%%%%%%%%%%%

ns = [];
function nbrs = nssearcher(seed,radius,dbg)
ts = tic(); if (nargin<3); dbg = 0; end
nbrs = rangesearch(ns,[x(seed),y(seed),z(seed)],radius); nbrs = nbrs{1};
n = length(nbrs); if (n>k); ii = randperm(n); nbrs = nbrs(ii(1:k)); end
if (dbg); fprintf('nssearcher (%s): %gs\n',method,toc(ts)); end
end

% bfs methods use adjtri %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bfs = [];  % bfs visited flags
hood = []; % accumulated neighborhood
md = [];   % mesh distance to seed
vdh = [];  % vert, diag, horiz edge lengths ((M-1)*(N-1))x3

mk = 0; % rolling flag avoids resetting whole bfs matrix for each search

% triangle neighbors lut: relative linear neighbor vertex indices in xyz for
% each of the six possible neighbor triangles of v
a = 1; b = nr+1; c = nr; d = -1; e = -nr-1; f = -nr;
tv = [a, b;  %0   e--d    
      b, c;  %1   |\3|\   
      c, d;  %2   |4\|2\  
      d, e;  %3   f--v--c
      e, f;  %4    \5|\1|
      f, a]; %5     \|0\|
             %       a--b

% edge neighbors lut: relative linear indices in vdh for each of the six
% possible neighbor triangles of v
cc = nr*nc; % stride from column to column in vdr
a = 0; b = cc; c = 2*cc; d = -1; e = -nr-1+cc; f = -nr+2*cc;
ev = [a, b;  %0   e--d     v-he-* 
      b, c;  %1   |\3|\    |\   | 
      c, d;  %2   |4\|2\   v d  | 
      d, e;  %3   f--v--c  e  e |
      e, f;  %4    \5|\1|  |   \|
      f, a]; %5     \|0\|  *----*
             %       a--b

function nbrs = trisearcher(seed,radius,dbg)
domd = ~isempty(md); % whether to use mesh distance
ts = tic(); if (nargin<3); dbg = 0; end
mk = mk+1; bfs(seed) = mk; % rolling visited flag
wf = seed; ii = 1; jj = 1; nh = 1; % initial wavefront is just the seed
if (domd); md(seed) = 0; end % mesh distance to seed is zero
while (ii<=jj) % while wavefront nonempty
  hh = ii:jj; hood(hh) = wf; nh = jj; % save wavefront
  ii = jj+1; if (ii>k); break; end % out of space?
  bfs(wf) = mk; tt = adjtri(wf); % mark visited, get bitmasks of adjacent tris
  nn = length(wf); nf = nan(nn,12); % space for up to 12 new verts for each old
  if (domd); nfd = nan(nn,12); end % corresponding mesh distances
  for t=0:5 % for each possible adjacent triangle
    kk = bitand(tt,2^t)~=0; nk = sum(kk); % wavefront verts which have adj tri t
    if (nk==0); continue; end % avoids exception in next line
    nf(kk,2*t+1:2*t+2) = repmat(wf(kk),1,2)+repmat(tv(t+1,:),nk,1);
    if (domd)
      nfd(kk,2*t+1:2*t+2) = repmat(md(wf(kk)),1,2)+... % mesh dist to old vert
          vdh(repmat(wf(kk),1,2)+repmat(ev(t+1,:),nk,1)); % edge dist to new
    end
  end
  wf = nf(:); if (domd); wfd = nfd(:); end % vectorize
  kk = ~isnan(wf); wf = wf(kk); if (domd); wfd = wfd(kk); end % clean
  kk = bfs(wf)~=mk; wf = wf(kk); if (domd); wfd = wfd(kk); end % cull visited
  [wf,kk] = unique(wf); if (domd); wfd = wfd(kk); end % uniquify
  if (~domd) % calculate direct distances from seed
    wfd = sqrt(sum(([x(wf),y(wf),z(wf)]-...
                    repmat([x(seed),y(seed),z(seed)],length(wf),1)).^2,...
                   2));
  end
  wf = wf(wfd<=radius); % cull by distance
  jj = min(ii+length(wf)-1,k); % jj<ii iff wf empty
  wf = wf(1:(jj-ii+1));
  if (domd); md(wf) = wfd(1:length(wf)); end % save mesh distances to wf verts
end
nbrs = hood(1:nh);
if (dbg); fprintf('trisearcher (%s): %gs\n',method,toc(ts)); end
end

function el = edgelengths()
% v-he-*
% |\   |
% v d  |
% e  e |
% |   \|
% *----*

ve = [sqrt((x(2:nr,:)-x(1:nr-1,:)).^2+...          
           (y(2:nr,:)-y(1:nr-1,:)).^2+...          
           (z(2:nr,:)-z(1:nr-1,:)).^2);
      nan(1,nc)];

de = [sqrt((x(2:nr,2:nc)-x(1:nr-1,1:nc-1)).^2+...
           (y(2:nr,2:nc)-y(1:nr-1,1:nc-1)).^2+...
           (z(2:nr,2:nc)-z(1:nr-1,1:nc-1)).^2),   nan(nr-1,1);
      nan(1,nc)];

he = [sqrt((x(:,2:nc)-x(:,1:nc-1)).^2+...          
           (y(:,2:nc)-y(:,1:nc-1)).^2+...          
           (z(:,2:nc)-z(:,1:nc-1)).^2),   nan(nr,1)];

el = [ve(:), de(:), he(:)];

end

% init datastructures %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

switch (method)
  case {'kdtree','exhaustive'}; 
    ns = createns([x(:),y(:),z(:)],'NSMethod',method);
    searcher = @nssearcher;
  case {'bfsdd','bfsmd'};
    bfs = uint32(zeros(sz)); hood = uint32(zeros(k,1));
    if (strcmpi(method,'bfsmd')); md = zeros(sz); vdh = edgelengths(); end
    searcher = @trisearcher;
  otherwise; error('unexpected method %s',method);
end

if (dbg); fprintf('samplesearcher: %gs\n',toc(tstart)); end

end
