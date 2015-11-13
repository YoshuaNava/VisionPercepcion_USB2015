function testpatchsample(tpi)
% testpatchsample(tpi) tests patchsample() on each patch type in tpi
%
%   See testpatches() for the meaning of tpi.
%
%   testpatchsample() uses tpi=1:10
%
% Copyright (C) 2013 Marsette A. Vona


cx = -1; cy = -1; cz = 6; % center of projection

% frustum
fh = pi/3; fv = pi/3; fp = -[cx, cy, cz]; fp = fp./norm(fp); fu = [0 1 0];
nh = 20; nv = 20;
[mx, my, mz] = samplefrustum(fh, fv, nh, nv, fp, fu);

if (nargin < 1); tpi=1:10; end

persistent fig;
if (isempty(fig)); fig = figure(); end
if (ishandle(fig)); clf(fig); else figure(fig); end % make current w/o focus
set(fig,'Color','w');

  function ok = test(p)
  r = patchsample(p,mx,my,mz,'cx',cx,'cy',cy,'cz',cz);
  sampleplot('mx',mx,'my',my,'mz',mz,'r',r,'cx',cx,'cy',cy,'cz',cz,...
             'frustum',[fh, fv, fp, fu]);
  ok = 1;
  end

testpatches(@test, tpi, 1);

end
