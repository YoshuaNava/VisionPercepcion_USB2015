function testpatcharea(tpi)
%testpatcharea(tpi) computs boundary and triangulation area of each patch in tpi
%
%   See testpatches() for the meaning of tpi (defaults to 1:10).
%
% Copyright (C) 2013 Marsette A. Vona

if (nargin < 1); tpi=1:10; end

  function ok = test(p)
 
  p = patchchk(p,'cl',1);

  ts = tic();
  p = patchchk(p,'ke',1,'gb',1,'ga',1);
  sec = toc(ts);

  str = patchprint(p); str(str==sprintf('\n')) = ' ';
  fprintf('patch params: %s\n', str);
  fprintf('boundary area: %g; triangulation area: %g; time %gs\n',...
          p.ba, p.ta, sec);

  ok = 1;

  end

testpatches(@test,tpi,0,{}); 

end
