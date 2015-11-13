function o = drawtriad(p,da,as,aas,das,aw)
% o = drawtriad(p,da,as,aas,das,aw) draws or updates local frame axis triad
%
%   p is the hggroup or hgtransform parent handle
%   
%   da is whether to draw the triad (if not any existing triad is deleted)
%
%   as is the triad scale
%
%   aas is an alt string for as to imply auto
%
%   das is the value for auto as
%
%   aw is the triad line width
%
%   Returns a handle to the hggroup containing the triad.
%
% Copyright (C) 2013 Marsette A. Vona

o = findobj(get(p,'Children'),'flat','Tag','triad');

if (da)
  if (ischar(as)&&strcmpi(aas,as)); as = das; end
  coords = {[[0;as],[0;0],[0;0]], [[0;0],[0;as],[0;0]], [[0;0],[0;0],[0;as]]};
  colors = {'r','g','b'};
  if (isempty(o))
    o = hggroup('Parent',p,'Tag','triad');
    for i=1:3; line('Parent',o); end
  end
  ll = get(o,'Children');
  for i=1:3
    c = coords{i};
    set(ll(i),'Xdata',c(:,1), 'Ydata',c(:,2), 'Zdata',c(:,3),...
              'Color',colors{i},'LineWidth',aw);
  end
else delete(findobj(o)); end

end
