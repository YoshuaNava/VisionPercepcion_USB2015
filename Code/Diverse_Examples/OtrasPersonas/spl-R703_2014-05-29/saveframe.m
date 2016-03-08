function saveframe(fn,h)
% saveframe(fn,h) saves current axes or figure image h to file fn
%
%   The image format is implied by the filename extension, see imwrite().
%
%   If h is omitted it defaults to the current axes.
%
% Copyright (C) 2013 Marsette A. Vona

if (nargin<2); h = gca(); end

imwrite(frame2im(getframe(h)),fn);

end
