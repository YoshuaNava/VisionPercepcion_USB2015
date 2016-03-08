function m = rexp(r)
% m = rexp(r) 3x3 rotation matrix corresponding to 3x1 or 1x3 rotation vec.
%
%  Uses Rodrigues' rotation formula.  Numerically stable as norm(r) approaches
%  zero.
%
% Copyright (C) 2013 Marsette A. Vona

%~20% speedup by skipping argchecks
%if (~isnumeric(r) || ~isvector(r) || (length(r)~=3))
%  error('r must be 3x1 or 1x3');
%end

m = eye(3);
t = norm(r);

if (t > 0)
  
  %rx = cpm(r);
  %a = alpha(t); b = beta(t);
  
  %~75% speedup by doing the calcs here
  rx = [0, -r(3), r(2); r(3), 0, -r(1); -r(2), r(1), 0];
  ath = sqrt(sqrt(eps(1)))*10; bth = sqrt(sqrt(eps(1/2)))*10;
  a = 1; if (t>ath); a = sin(t)/t; elseif (t>0); a = 1-t^2/6; end
  b = 1/2; if (t>bth); b = (1-cos(t))/t^2; elseif (t>0); b = (1/2-t^2/24); end
  
  m = m+rx.*a+rx*rx.*b;
  
end

end
