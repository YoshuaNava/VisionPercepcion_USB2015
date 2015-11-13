function o = drawcurves(p,dl,tag,x,y,z,lc,lw)
% o = drawcurves(p,dl,tag,x,y,z,lc,lw) draws or updates curves
%
%   p is the hggroup or hgtransform parent handle
%   
%   dl is whether to draw the curves (if not any existing ones are deleted)
%
%   tag is the Tag string property for the generated group
%
%   x,y,z are Nx1 or 1xN cell arrays such that each (x{i}, y{i}, z{i}) are three
%   equal sized matrices with coordinates for the vertices of one contiguous
%   open curve
%
%   lc is the line color
%
%   lw is the line width
%
%   Returns a handle to the hggroup containing the curves.
%
% Copyright (C) 2013 Marsette A. Vona

o = findobj(get(p,'Children'),'flat','Tag',tag);

if (dl)
  n = length(x);
  if (isempty(o)); o = hggroup('Parent',p,'Tag',tag); end
  ll = get(o,'Children'); nll = length(ll);
  if (nll<n); for i=nll+1:n; line('Parent',o); end
  elseif (nll>n); for i=nll:-1:n+1; delete(ll(i)); end; end
  ll = get(o,'Children');
  for i=1:n
    set(ll(i),'XData',x{i},'YData',y{i},'ZData',z{i},...
              'Color',lc,'LineWidth',lw);
  end
else delete(findobj(o)); end

end
