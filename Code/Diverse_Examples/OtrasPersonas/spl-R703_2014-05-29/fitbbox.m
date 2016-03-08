function fitbbox(a,grow)
% fitbbox(a,grow) fit axes limits to bounding box
%
%   a is the axes to configure, default gca()
%
%   grow is the amount to grow the box in every dir, default 1
%
%   see hgbbox()
%
% Copyright (C) 2013 Marsette A. Vona

if (nargin<2); grow = 1; end
if (nargin<1); a = gca(); end

% work around possible actual matlab bug: objbounds() does not correctly
% handle hgtransforms
%
% http://www.mathworks.com/matlabcentral/newsreader/view_thread/295878 
b = hgbbox(a,grow);
if (b(2)>b(1)); xlim(a,b(1:2)); end
if (b(4)>b(3)); ylim(a,b(3:4)); end
if (b(6)>b(5)); zlim(a,b(5:6)); end

end
