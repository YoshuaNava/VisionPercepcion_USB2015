function testrexp()
%testrexp tests rexp and related functions
%
%   The tests are randomized and non-interactive.  Execution results in an error
%   if any test results in a difference larger than a predetermined threshold.
%
%   The main functions are timed and the results displayed at the end of a
%   successful test run.  It may be necessary to run testrexp multiple times to
%   get meaningful timing results, in part due to JIT.
%
% Copyright (C) 2013 Marsette A. Vona

tol = 1e-9;
tol2 = 1e-3;
tol3 = 5e-2;
tol4 = 1e-5;
N = 100;
vecs = [[0;0;0],[2*pi;0;0],[pi;0;0],[0;pi;0],[0;0;pi]];
%vecs = [];
testpi = 1;
mags = [1e-3 1 10 1e3];

ncrexp = 0; srexp = 0;
ncdrexp = 0; sdrexp = 0;
ncrlog = 0; srlog = 0;
ncaltrexp = 0; saltrexp = 0;
ncaltdrexp = 0; saltdrexp = 0;
ncnumdrexp = 0; snumdrexp = 0;
ncdrlog = 0; sdrlog = 0;
ncnumdrlog = 0; snumdrlog = 0;
ncrreparam = 0; srreparam = 0;
ncrcanon2 = 0; srcanon2 = 0;
ncdrcanon2 = 0; sdrcanon2 = 0;
ncnumdrcanon2 = 0; snumdrcanon2 = 0;
ncrtoq = 0; srtoq = 0;
ncqtor = 0; sqtor = 0;
ncdummy = 0; sdummy = 0;

nt = 0;

% run all tests for a given rotation vector r
  function test(r)
    
  foo = time(@dummy,r);
  
  m = time(@rexp,r);
  qm = time(@altrexp,r);
  check(m,qm,tol,'qm',r,nt);
  
  cr = time(@rreparam,r);
  assert(norm(cr)<=pi);
  cm = time(@rexp,cr);
  check(m,cm,tol4,'cm',r,nt);
  
  rr = time(@rlog,m);
  check(cr,rreparam(rr),tol,'rlog',r,nt);
  
  dm = time(@drexp,r);
  ndm = time(@numdrexp,r);
  qdm = time(@altdrexp,r);
  check(qdm,ndm,tol2,'qdm-ndm',r,nt);
  check(dm,ndm,tol2,'ndm',r,nt);
  check(dm,qdm,tol2,'qdm',r,nt);
  
  cm = time(@rexp,cr);
  dr = time(@drlog,cm);
  ndr = time(@numdrlog,cm);
  if (abs(norm(cr)-pi)>1e-3) check(dr,ndr,tol3,'ndr',r,nt); end % see numdrlog
  
  % drlog(rexp(cr),'m')*drexp(cr,'m') is the Jacobian of rlog(rexp(cr)), which
  % should be identity by definition
  dmm = drexp(cr,'m');
  drm = time(@drlog,cm,'m');
  check(drm*dmm,eye(3,3),tol4,'drm*dmm',r,nt);
  
  for i=1:3
    cm = time(@rexp,time(@rcanon2,r,i,3));
    check(m(:,i),cm(:,i),tol2,sprintf('cm%d',i),r,nt);
    if (abs(norm(cr)-pi)>1e-2)
      dr2 = time(@drcanon2,cr,i,3);
      ndr2 = time(@numdrcanon2,cr,i,3);
      check(dr2,ndr2,tol3,'ndr2',r,nt);
    end
    %  dr2 = time(@drcanon2,cr,i,2); a = zeros(3,1); a(i) = 1;
    %  check(pinv(dr2)*dr2,eye(3,3)-a*a',tol3,'dr2^+*dr2',r,nt);
  end
  
  q = time(@rtoq,r);
  rr = time(@qtor,q);
  check(cr,rr,tol,'rtoq',r,nt);

  nt = nt+1;
  end

% explicitly test some specific vectors
[~,nv] = size(vecs);
for i=1:nv
  v = vecs(:,i);
  fprintf('testing r=%s\n',mat2str(v,6));
  test(v);
  fprintf('testing r=%s\n',mat2str(-v,6));
  test(-v);
  for j=1:10
    test(v+rand(3,1)*eps);
    test(v+rand(3,1)*eps*10);
    test(v+rand(3,1)*eps*100);
    test(v+rand(3,1)*eps*1000);
    test(v+rand(3,1)*sqrt(eps));
    test(v+rand(3,1)*sqrt(eps)*10);
    test(v+rand(3,1)*sqrt(eps)*100);
    test(v+rand(3,1)*sqrt(eps)*1e3);
    test(v+rand(3,1)*sqrt(eps)*1e4);
    test(v+rand(3,1)*sqrt(eps)*1e5);
  end
end

if (testpi)
  fprintf('testing %d random inputs with magnitude %g\n', N, pi);
  for i=1:N;
    r = (rand(3,1)-0.5*ones(3,1));
    if (norm(r)>sqrt(eps))
      r = r/norm(r); r = r*pi;
      test(r);
    end
  end
end

% run batches of randomized tests
for mag=mags
  fprintf('testing %d random inputs with component magnitude up to %g\n',N,mag);
  for i=1:N; test((rand(3,1)-0.5*ones(3,1))*mag); end
end

nm = {'rexp','drexp','altrexp','altdrexp','numdrexp', ...
      'rlog','drlog','numdrlog',...
      'rreparam','rcanon2','drcanon2','numdrcanon2',...
      'rtoq','qtor',...
      'dummy'};
for i=1:length(nm)
  sec = eval(['s' nm{i}]); nc = eval(['nc' nm{i}]);
  if (~strcmpi(nm{i},'dummy')) sec = sec-sdummy; end
  fprintf('%s avg %g ms per call\n', nm{i}, (sec*1000)/nc);
end

end

function o = time(f,varargin)
tstart=tic();
o = f(varargin{:});
fn = func2str(f);
evalin('caller',sprintf('s%s=s%s+%g;',fn,fn,toc(tstart)));
evalin('caller',sprintf('nc%s=nc%s+1;',fn,fn));
end

function o = dummy(foo)
o = foo;
end

% compare results
function check(m1, m2, t, m, r, nt)
if (any(isnan(m1(:)))||any(isnan(m2(:))))
  m1, m2
  assignin('base','badr',r);
  assignin('base','badm1',m1); assignin('base','badm2',m2);
  error('test %d %s nan for r=[%g %g %g]', nt, m, r(1), r(2), r(3));
end
if (any(isinf(m1(:)))||any(isinf(m2(:))))
  m1, m2
  assignin('base','badr',r);
  assignin('base','badm1',m1); assignin('base','badm2',m2);
  error('test %d %s inf for r=[%g %g %g]', nt, m, r(1), r(2), r(3));
end
n = norm(m1(:)-m2(:));
th = t*max(1,abs(max([m1(:);m2(:)])));
if (n > th)
  m1, m2
  assignin('base','badr',r);
  assignin('base','badm1',m1); assignin('base','badm2',m2);
  error('test %d %s diff norm %g > %g for r=[%g %g %g]',...
        nt,m,n,th,r(1),r(2),r(3));
end
end

% quaternion expmap
function [qw qx qy qz] = quat(r,t,t2)
if (t > 0) q = [cos(t2) (sin(t2)*r(:)/t)']; else q = [1 0 0 0]; end
qc = num2cell(q); [qw qx qy qz] = qc{:};
end

% quaternion to 3x3 rotation matrix
function m = qm(qw, qx, qy, qz)
m = [1-2*qy^2-2*qz^2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw;
     2*qx*qy+2*qz*qw, 1-2*qx^2-2*qz^2, 2*qy*qz-2*qx*qw;
     2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx^2-2*qy^2];
end

% alternate to rexp() based on quaternions
function m = altrexp(r)
t = norm(r); t2 = t/2;
[qw qx qy qz] = quat(r,t,t2);
m = qm(qw, qx, qy, qz);
end

% alternate to drexp() based on quaternions
function m = altdrexp(r)

t = norm(r); t2 = t/2; tt = t*t; s2 = sin(t/2); c2 = cos(t/2);

if (t > sqrt(sqrt(eps(1/2)))*10) A = s2/t; else A = 1/2-tt/48; end
if (t > sqrt(sqrt(eps(1/24)))*10) B = (c2/2-A)/tt; else B = (tt/40-1)/24; end

dq = zeros(4,3);

for j=1:3; dq(1,j)=-(r(j)/2)*A; end
for i=1:3
  for j=1:3
    if (i==j) dq(i+1,j)=A+r(j)*r(j)*B;
    else dq(i+1,j)=r(i)*r(j)*B; end
  end
end

[qw qx qy qz] = quat(r,t,t2);

dmdqw = [ 0,    -2*qz, 2*qy; 2*qz,  0,    -2*qx; -2*qy, 2*qx,  0];
dmdqx = [ 0,     2*qy, 2*qz; 2*qy, -4*qx, -2*qw;  2*qz, 2*qw, -4*qx];
dmdqy = [-4*qy,  2*qx, 2*qw; 2*qx,  0,     2*qz; -2*qw, 2*qz, -4*qy];
dmdqz = [-4*qz, -2*qw, 2*qx; 2*qw, -4*qz,  2*qy;  2*qx, 2*qy,  0];

m = zeros(3,3,3);
for k=1:3
  m(:,:,k) = dmdqw*dq(1,k) + dmdqx*dq(2,k) + dmdqy*dq(3,k) + dmdqz*dq(4,k);
end
end

% numeric alternate to drexp()
function j = numdrexp(r)
nde = 1e-6;
r0 = r; m0 = rexp(r0);
j = zeros(3,3,3);
for k=1:3; r = r0; r(k) = r(k)+nde; j(:,:,k) = (rexp(r)-m0)/nde; end
end

% numeric alternate to drlog()
% very difficult to get right around theta=pi
function j = numdrlog(m)
nde = 1e-6;
m0 = m; r0 = rlog(m0);
j = zeros(3,3,3);
for r=1:3
  for c=1:3
    %    fprintf('r=%d, c=%d, ',r,c);
    mp = m0; mp(r,c) = mp(r,c)+nde; rp = rlog(mp);
    rp = rreparam(rlog(mp),r0);
    %    mn = m0; mn(r,c) = mn(r,c)-nde; rn = rlog(mn);
    %    rn = rreparam(rlog(mn),r0);
    %    dp = rp-r0; dn = rn-r0;
    %    if (sum(dp.*dp)<sum(dn.*dn)) dr = rp-r0; dm = nde;
    %    else dr = rn-r0; dm = -nde; end
    dr = rp-r0; dm = nde;
    %    dr = rn-r0; dm = -nde;
    j(r,c,:) = reshape(dr/dm,1,1,3);
    %    j(r,c,:) = reshape((rr-r0)/nde,1,1,3);
    %    j(r,c,:) = reshape((rp-rn)/(2*nde),1,1,3);
  end
end
end

% numeric alternate to drcanon2()
function j = numdrcanon2(r,p,d)
nde = 1e-6;
r0 = r; rc0 = rcanon2(r,p,d);
j = zeros(d,3);
for k=1:3; r = r0; r(k) = r(k)+nde; j(:,k) = (rcanon2(r,p,d)-rc0)/nde; end
end

