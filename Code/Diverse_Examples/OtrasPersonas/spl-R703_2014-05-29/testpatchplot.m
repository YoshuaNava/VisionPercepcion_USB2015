function testpatchplot(tpi, varargin)
%testpatchplot(tpi) tests patchplot() on each patch type in tpi
%
%   See testpatches() for the meaning of tpi (defaults to 1:10).
%
%   Extra args are passed on to patchplot().
%
% Copyright (C) 2013 Marsette A. Vona

if (nargin < 1); tpi=1:10; end

persistent fig;
if (isempty(fig)); fig = figure(); end
if (ishandle(fig)); clf(fig); else figure(fig); end % make current w/o focus
set(fig,'Color','w');

if (nargin<2); testpatches(@(p)(1),tpi,1,{}); 
else testpatches(@(p)(1),tpi,1,varargin{:}); end

end
