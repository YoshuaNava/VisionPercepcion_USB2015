function [xl xu yl yu zl zu] = hgbbox(hh,grow)
% [xl xu yl yu zl zu] = hgbbox(hh)
%
%   Recursively calculates the spatial bounding box of the geometry contained in
%   a vector of hgtransform handles.  Like the objbounds() standard function
%   but correctly handles hgtransforms.
%
%   See http://www.mathworks.com/matlabcentral/newsreader/view_thread/295878 
%
%   Optional argument grow is a the factor by which to grow the returned
%   bounds in every direction, if omitted no growth is applied.
%
%   Returns either a single 1x6 vector or 6 scalars.
%
% Copyright (C) 2013 Marsette A. Vona

dbg = 0;

if (nargin<2); grow = 1; end

  function msg(varargin)
  if (dbg); fprintf(varargin{:}); end
  end

% init bounding box
xl = inf; xu = -inf; yl = inf; yu = -inf; zl = inf; zu = -inf;

  function union(bb)
  if (isempty(bb)); msg('bb empty\n'); return; end
  % union bounding box bb to [xl xu yl yu zl zu]
  msg('unioning %d %d %d %d %d %d\n',bb(1),bb(2),bb(3),bb(4),bb(5),bb(6));
  xl = min(xl,bb(1)); xu = max(xu,bb(2));
  yl = min(yl,bb(3)); yu = max(yu,bb(4));
  zl = min(zl,bb(5)); zu = max(zu,bb(6));
  end

  function bb = xform(bb,m)
  % apply 4x4 homogenous transform m to bounding box bb
  lu = reshape(bb,2,3); bb = [inf -inf inf -inf inf -inf];
  for i=0:7 % xform and union each corner
    c = m*[lu(bitget(i,1)+1,1);lu(bitget(i,2)+1,2);lu(bitget(i,3)+1,3);1];
    bb(1) = min(c(1),bb(1)); bb(2) = max(c(1),bb(2));
    bb(3) = min(c(2),bb(3)); bb(4) = max(c(2),bb(4));
    bb(5) = min(c(3),bb(5)); bb(6) = max(c(3),bb(6));
  end
  end

  function b = istype(h,t)
  % check if handle h has type t
  b = strcmpi(t,get(h,'Type'));
  end

  function cc = chldrn(h)
  % get children of handle h
  cc = get(h,'Children');
  end

for h = hh(:)' % for loop iterates over columns
  msg('h=%d type=%s tag=%s\n',h,get(h,'Type'),get(h,'Tag'));
  msg('bb before %d %d %d %d %d %d\n',xl,xu,yl,yu,zl,zu);
  if (istype(h,'hgtransform')) union(xform(hgbbox(chldrn(h)),get(h,'Matrix')));
  elseif (istype(h,'hggroup')||istype(h,'axes')) union(hgbbox(chldrn(h)));
  else union(objbounds(h)); end
  msg('bb after %d %d %d %d %d %d\n',xl,xu,yl,yu,zl,zu);
end

if (any(isinf([xl xu]))); xl = 0; xu = 0; end
if (any(isinf([yl yu]))); yl = 0; yu = 0; end
if (any(isinf([zl zu]))); zl = 0; zu = 0; end

grow = grow-1;
gx = (xu-xl)*grow; xl = xl-gx; xu = xu+gx;
gy = (yu-yl)*grow; yl = yl-gy; yu = yu+gy;
gz = (zu-zl)*grow; zl = zl-gz; zu = zu+gz;

if (nargout<=1); xl = [xl xu yl yu zl zu]; end

end
