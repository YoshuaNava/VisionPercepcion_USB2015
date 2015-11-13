function [r, dr] = rlog(m,fmt)
%r = rlog(m) 3x1 orientation vector corresponding to 3x3 rotation matrix
%
%  Return is always canonical, i.e. the invariant rreparam(rlog(m)) ==
%  rlog(m) should always hold.
%
%  Optional output dr is the Jacobian of the mapping in the format specified
%  by the optional input fmt, see drlog() for details of both.
%
%  Numerically stable as m approaches identity.
%
% Copyright (C) 2013 Marsette A. Vona

v = [m(3,2)-m(2,3);m(1,3)-m(3,1);m(2,1)-m(1,2)];
cc = 0.5*(trace(m)-1); if (cc>1); cc = 1; elseif (cc<-1); cc=-1; end
ss = 0.5*norm(v); if (ss<0); ss = 0; elseif (ss>1); ss = 1; end
t = atan2(ss,cc);

%ijk = 123, 231, or 312 st m(i,i) is max in diag(m)
[~,i]=max(diag(m)); j = mod(i,3)+1; k = mod(j,3)+1;

d2 = 1+m(i,i)-m(j,j)-m(k,k);

th = 1e-4;
if (d2>th)
  
  %  fprintf('large, i=%d\n',i);
  %  fprintf('large, t=%d\n',t);
  
  d = sqrt(d2);
  
  th = sqrt(eps(1))*100;
  if (t>th); gg = t/sqrt(3-trace(m));
  else gg = sqrt(12/(12-t*t));
  end
  
  r = zeros(3,1);
  
  r(i) = gg*d; ggd = gg/d;
  r(j) = ggd*(m(j,i)+m(i,j));
  r(k) = ggd*(m(k,i)+m(i,k));
  
  th = sqrt(eps(1))*100;
  f = 1;
  if (t<(pi-th))
    p = cross(r,[0;0;1]); if (sum(p.*p)<0.25); p=cross(r,[0;1;0]); end
    if ((m*p)'*cross(r,p)<0); f = -1; end
  elseif (sum(rreparam(r).*r)<0); f = -1; end
  
  d = f*d; ggd = f*ggd; r = r*f;
  
  if (nargout>1)
    dr = zeros(3,3,3);
    rh = r/t;
    ds = 0.5*cpm(rh); dc = 0.5*eye(3,3); dt = cc*ds-ss*dc;
    ee = zeros(3,3); ee(i,i) = 1; ee(j,j) = -1; ee(k,k) = -1;
    gdd = 0.5*gg*ee/d;
    ddg = rh(i)*(dt+0.5*t*eye(3,3)/(3-trace(m)));
    dr(:,:,i) = gdd + ddg;
    ee = zeros(3,3); ee(j,i) = 1; ee(i,j) = 1;
    dr(:,:,j) = ggd*ee+(m(j,i)+m(i,j))*(ddg-gdd)/d2;
    ee = zeros(3,3); ee(k,i) = 1; ee(i,k) = 1;
    dr(:,:,k) = ggd*ee+(m(k,i)+m(i,k))*(ddg-gdd)/d2;
  end
  
else
  
  %  fprintf('small, t=%d\n',t);
  
  %  th = sqrt(sqrt(eps(1)))*10;
  th = sqrt(eps(1))*100;
  if (t>th); aa = t/ss; else aa = 6/(6-t^2); end
  r = 0.5*aa*v;
  
  if (nargout>1)
    
    if (t>th)
      b = 0.5*(ss-t*cc)/(ss*ss);
      rh = r/t;
      %      th = sqrt(eps(1))*100;
      %      if (ss>th)
      dt = 0.5*(cc*cpm(rh)-ss*eye(3,3));
      %      else dt = 0.5*(cc*(ones(3,3)-eye(3,3))-ss*eye(3,3)); end
    else
      b = t/12; dt = 0.5*(cc*(ones(3,3)-eye(3,3))-ss*eye(3,3));
      %      b = 0; dt = zeros(3,3);
    end
    
    drx = zeros(3,3,3);
    drx(2,3,1) = -1; drx(3,2,1) = 1;
    drx(1,3,2) = 1; drx(3,1,2) = -1;
    drx(1,2,3) = -1; drx(2,1,3) = 1;
    
    dr = zeros(3,3,3);
    for i=1:3; dr(:,:,i) = 0.5*aa*drx(:,:,i)+v(i)*b*dt; end
    
  end % if output dr
end % if t small

if ((nargout>1)&&(nargin>1)&&(fmt=='m'))
  dr = [reshape(dr(:,:,1),1,9);reshape(dr(:,:,2),1,9);reshape(dr(:,:,3),1,9)];
end

end

% % robust at t~=pi, but about 20x slower
%[~,~,v]=svd(m-eye(3,3)); v = v(:,3);
%p = cross(v,[0;0;1]); if ((p.*p)<0.25) p=cross(v,[0;1;0]); end
%pp = m*p; if ((pp'*cross(v,p))<0) t = -t; end
%r = v*t;

%v = [m(3,2)-m(2,3);m(1,3)-m(3,1);m(2,1)-m(1,2)];
%cc = 0.5*(trace(m)-1); if (cc>1) cc = 1; elseif (cc<-1) cc=-1; end
%ss = 0.5*norm(v); if (ss<0) s = 0; elseif (ss>1) ss = 1; end
%
%t = atan2(ss,cc);
%
%th = sqrt(eps(1))*100;
%if (t<(pi-th))
%  th = sqrt(sqrt(eps(1)))*10;
%  if (t>th) aa = t/ss; else aa = 6/(6-t^2); end
%  r = 0.5*aa*v;
%else % t is near pi
%  bth = sqrt(sqrt(eps(1/2)))*10;
%  b = 1/2; if (t > bth) b = (1-cc)/t^2; elseif (t > 0) b = (1/2-t^2/24); end
%  d = m(1,1); e=m(2,2); f = m(3,3);
%  rx = sqrt(max(1+d-e-f,0)/(2*b));
%  ry = sqrt(max(1-d+e-f,0)/(2*b));
%  rz = sqrt(max(1-d-e+f,0)/(2*b));
%  rr = [rx, ry, rz; -rx, ry, rz; rx,-ry, rz; rx, ry,-rz]';
%  rr2 = (m-eye(3,3))*rr;
%  [~,i] = min(sum(rr2.*rr2));
%  r = rr(:,i);
%  p = cross(r,[0;0;1]); if (sum(p.*p)<0.25) p=cross(r,[0;1;0]); end
%  pp = m*p; if ((pp'*cross(r,p))<0) r = -r; end
%end
