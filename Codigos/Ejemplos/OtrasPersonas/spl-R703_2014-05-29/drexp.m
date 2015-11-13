function j = drexp(r,fmt)
%j = drexp(r) Jacobian of rexp(r)
%
%  The optional argument fmt controls the format of the output.  It may either
%  be 't' (default) or 'm'.  In the former case the output is a [3x3]x3 tensor
%  stored as a Matlab 3D array; j(:,:,i) is the Jacobian of exp(r) with respect
%  to r(i).  In the latter case the output is a 9x3 matrix, where j(:,i) is the
%  Jacobian of (exp(r))(:) with respect to r(i).
%
%  Numerically stable as norm(r) approaches zero.
%
% Copyright (C) 2013 Marsette A. Vona

t = norm(r);

%rx = cpm(r);
%a = alpha(t); b = beta(t);
%da = dalpha(t); db = dbeta(t);

%~80% speedup by doing the calcs here
rx = [0, -r(3), r(2); r(3), 0, -r(1); -r(2), r(1), 0];
a = 1; b = 1/2; da = -1/3; db = -1/12;
ath = sqrt(sqrt(eps(a)))*10; bth = sqrt(sqrt(eps(b)))*10;
dath = sqrt(sqrt(eps(da)))*10; dbth = sqrt(sqrt(eps(db)))*10;
if (t >= ath); a = sin(t)/t; elseif (t > 0); a = 1-t^2/6; end
if (t >= bth); b = (1-cos(t))/t^2; elseif (t > 0); b = (1/2-t^2/24); end
if (t >= dath); da = (t*cos(t)-sin(t))/t^3; elseif (t > 0); da = t^2/30-1/3; end
if (t >= dbth); db = (t*sin(t)+2*cos(t)-2)/t^4;
elseif (t > 0); db = t^2/180-1/12; end

rx2 = rx*rx;

%making drx persistent actually seems to hurt performance
%persistent drx;
%if (isempty(drx))
drx = zeros(3,3,3);
drx(2,3,1) = -1; drx(3,2,1) = 1;
drx(1,3,2) = 1; drx(3,1,2) = -1;
drx(1,2,3) = -1; drx(2,1,3) = 1;
%end

j = zeros(3,3,3);
for i=1:3;
  drxi=drx(:,:,i);
  j(:,:,i) = rx*da*r(i)+drxi*a+rx2*db*r(i)+(rx*drxi+drxi*rx)*b;
end

if ((nargin>1)&&(fmt=='m')); j = reshape(j,9,3); end

end
