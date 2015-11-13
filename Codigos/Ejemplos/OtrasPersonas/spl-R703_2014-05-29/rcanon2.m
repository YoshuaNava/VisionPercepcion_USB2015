function [r2,dr2] = rcanon2(r,p,d)
%r2 = rcanon2(r,p) canonical 2DoF orientation vector holding axis p fixed
%
%   The argument p may be 'x', 'y', or 'z'; or, equivalently, 1, 2, or 3.  If
%   omitted it defaults to 3.  r must be 3x1.
%
%   The argument d may be 2 or 3.  If omitted it defaults to 2.
%
%   The dx1 return orientation vector r2 is found such that r2 = eye(d,3)*rr,
%   rr(p) = 0 and R2(:,p) = R(:,p), where R2 = rexp(rr) and R = rexp(r).
%
%   Optional output dr is the dx3 Jacobian of the mapping, see drcanon2() for
%   details.
%
% Copyright (C) 2013 Marsette A. Vona

%~TBD% speedup by skipping argchecks
%if (~isnumeric(r) || ~isvector(r) || (length(r)~=3))
%  error('r must be 3x1 or 1x3');
%end

if (nargin<3); d = 2; end

if (nargin<2); p = 3;
else
  if (ischar(p)&&(length(p)==1)); p = find('xyz'==p); end
  if (~isnumeric(p)||~isscalar(p)||(p<1)||(p>3))
    error('p must be x, y, z, or 1, 2, 3');
  end
end

a = zeros(3,1); a(p) = 1;

r = rreparam(r); R = rexp(r); v = R(:,p);

r2 = cross(a,v);

ss = norm(r2); cc = v(p);
t = atan2(ss,cc);
th = sqrt(sqrt(eps(1)))*10;
if (t>th); aa = t/ss; else aa = 6/(6-t^2); end

th = sqrt(eps(1))*10;
if (t<(pi-th)); r2 = aa*r2;
else r2 = r; end

assert(abs(r2(p))<10*th,num2str(r2(p)));

vi = true(3,1); vi(p) = false;

if (nargout>1)
  I = eye(3,3);
  if (t<(pi-th))
    dR = drexp(r,'t');
    dRdrp = [dR(:,p,1),dR(:,p,2),dR(:,p,3)];
    if (ss>th) b = (t-ss*cc)/(ss*ss*ss);
    else b = (2/3)*(1/(1-t*t/6)); end
    P = I; P(p,p) = 0;
    dr2 = P*cpm(a)*(aa*I-R*a*a'*(b*R'*(I-a*a')+I))*dRdrp;
  else dr2 = I; end
  if (d~=3); dr2 = dr2(vi,:); end
end

if (d~=3); r2 = r2(vi); end

end
