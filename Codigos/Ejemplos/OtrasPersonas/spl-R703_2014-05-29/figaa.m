function f = figaa(k, r)
% figaa(k, r) antialias current figure with myaa from matlab FEX
%
%   k is the desired number of supersamples (must be in the range supported
%   by myaa)
%
%   r is the renderer to use
%
%   figaa(k) uses r='opengl'
%   figaa() uses k=4 and r='opengl'
%
%   Returns handle to newly opened figure window with the antialiased image
%   or [] if myaa was not available.
%
% Copyright (C) 2013 Marsette A. Vona

f = [];

if (~exist('myaa'))
  fprintf('figaa requires myaa from matlab FEX\n');
  return;
end

if (nargin < 1); k = 4; end;
if (nargin < 2); r = 'opengl'; end

ff = gcf();

rwas = get(ff,'renderer'); set(ff,'renderer',r);

fprintf('antialiasing current figure with %d supersamples...\n',k);
f = myaa(k);
%set(f,'MenuBar','figure');

set(ff,'renderer',rwas);

end
