function [res, sec] = patchresidual(p, varargin)
% res = patchresidual(p) computes a geometric residual of patch p
%
%   The parameter p is a (scalar) Matlab struct describing the patch. See
%   the documentation for the function patchchk() for details.
%
%   The residual evaluation is described in the paper "Sparse Surface Modeling
%   with Curved Patches" by Dimitrios Kanoulas and Marsette Vona, ICRA 2013.
%
%   TBD doc algorithms
%
%   INPUTS
%
%   The neighborhood of points is taken from the first available of the
%   following sources (unless the source is explicitly given with the option
%   'src', see OPTIONS below):
%
%   * explicitly given with the options 'x', 'y', 'z' in world frame, or
%   local frame if 'src' option is 'xyzl'
%
%   * from the patch field p.fitsd, which for a fitted patch is the subset of
%   data points in world frame used for surface fitting if the 'rsd' option was
%   specified in the call to patchfit()
%
%   OUTPUTS
%
%   The output res is the RMS residual computed by the algorithm given by the
%   'type' option, see OPTIONS below.
%
%   Optional output sec is the runtime of the function call in seconds.
%
%   OPTIONS
%
%   A list of (name,value) pairs may be given to set options.  Unrecognized 
%   names cause warnings.  If a name is given more than once the last-given 
%   (rightmost) value takes precedence.
%
%   Options 'x', 'y', 'z' (default []): coordinates of neighborhood points in
%   world frame, or local frame if 'src' option is 'xyzl'
%
%   Option 'src' (default 'auto'): One of 'auto', 'xyz', 'xyzl', 'fitsd'.
%   Explicitly requests that the neighborhood data be taken from the indicated
%   source.
%   
%   Option 'type' (default 'exact'): the residual type, one of 'exact',
%   'newton', 'vert', 'taubin1', 'taubin2'.  TBD Mahalanobis types.
%
%   Option 'newtonit' (default 30): if positive then the 'netwton' method uses
%   at most this many Newton iterations for root finding.
%
%   Option 'newtonstop' (default 0.01): if positive then the 'netwton' method 
%   stops iterating when a polynomial residual is below this value.
%
%   Option 'chkll' (default 0): whether to check Lagrange polynomial
%   solutions against the symbolic solver.
%
% Copyright (C) 2013 Dimitrios Kanoulas and Marsette Vona

tstart = tic();

tol = sqrt(eps); % tolerance for denominator zero checking

% process (name,value) options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
ix = []; iy = []; iz = []; src = 'auto'; type = 'exact'; chkll = 0;
newtonit = 15; newtonstop = 0.01;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'x'; ix = v; case 'y'; iy = v; case 'z'; iz = v;
      case 'src'; src = v; case 'type'; type = v; case 'chkll'; chkll = v;
      case 'newtonit'; newtonit = v; case 'newtonstop'; newtonstop = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

% get data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

iw = 0; % input in world frame?

x = []; y = []; z = [];
isauto = strcmpi(src,'auto');

if ((isauto&&~isempty(ix))||strcmpi(src,'xyz')||strcmpi(src,'xyzl'))
  x = ix; y = iy; z = iz; iw = ~strcmpi(src,'xyzl');
end

if (((isauto&&isempty(x))||strcmpi(src,'fitsd'))&&isfield(p,'fitsd'))
  x = p.fitsd(:,1); y = p.fitsd(:,2); z = p.fitsd(:,3); iw = 1;
end

%if (isempty(z)); error('no data'); end

sz = size(x); nd = sz(1)*sz(2);

if (any(size(y)~=sz)||any(size(z)~=sz))
  error('data must all be same size');
end

x = x(:); y = y(:); z = z(:); % vectorize

if (iw) % transform to local frame
  rrinv = rexp(-p.r);
  xyz = [x,y,z]*rrinv'+repmat((-rrinv*p.c(:))',nd,1);
  x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);
end

% determine parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = patchchk(p,'gk',1,'gf',1); % update curvatures and surface functions
kx = p.kx; ky = p.ky; kxx = kx*kx; kyy = ky*ky; kxy = kx*ky;

% functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  function sol = polysolve(coeffs,qx,qy,qz)
  % solve Lagrange polynomial with roots()
  % returns minimum corresponding squared distance
  % not vectorized

  sol = inf; ll = roots(coeffs);

  if (~isempty(ll)) % select real root

    for lr = ll'
      if (imag(lr)==0)
        s = lr*lr*((kx*qx/(1+lr*kx))^2 + (ky*qy/(1+lr*ky))^2 + 1);
        sol = min(sol,s);
      end
    end

    %q = [qx;qy;qz];
    %for j = 1:length(ll)
    %  pt = (eye(3)+ll(j)*diag([kx ky 0])) \ (q+ll(j)*[0;0;1]);
    %  if (~imag(pt))
    %    d2 = (q-pt)'*(q-pt);
    %    sol = min(d2, sol);
    %  end
    %end
          
    if (chkll) % check symbolic solution for ll
      sl = sym('sl');
      sll = solve(qx^2*kx/(1+sl*kx)^2 + qy^2*ky/(1+sl*ky)^2 - 2*(qz+sl));
      llr = double(sort(ll(imag(ll)==0)));
      sllr = double(sort(sll(imag(sll)==0)));
      if (any(abs(llr-sllr)>tol)); warning('llr~=sllr'); llr,sllr, end
    end
    
  end % select root

  end % polysolve()

  function o = newtonsolve(aa, bb, cc, dd, ee, ff, qx, qy, u) 
  % solve Lagrange polynomial by Newton iteration for initial guess u
  % returns squared distances corresponding to roots

  su = size(u);
  vi = true(su); p = zeros(su); dp = zeros(su);
  u2 = zeros(su); u3 = zeros(su); 
  u4 = zeros(su); u5 = zeros(su); 

  for i = 1:newtonit
    
    u2(vi) =  u(vi).*u(vi); u3(vi) = u2(vi).*u(vi);
    u4(vi) = u3(vi).*u(vi); u5(vi) = u4(vi).*u(vi);


    p(vi) = u5(vi).*aa(vi)+u4(vi).*bb(vi)+...
            u3(vi).*cc(vi)+u2(vi).*dd(vi)+u(vi).*ee(vi)+ff(vi);

    % converged or diverging
    if (newtonstop>0); vi = vi&(abs(p)>newtonstop); end
    if (i>1); di = vi&(abs(p)>abs(pwas)); u(di) = uwas(di); vi = vi&~di; end
    if (~any(vi)); break; end

    dp(vi) = 5*u4(vi).*aa(vi)+4*u3(vi).*bb(vi)+...
             3*u2(vi).*cc(vi)+2*u(vi).*dd(vi)+ee(vi);

    uwas = u; pwas = p; 

    u(vi) = u(vi)-p(vi)./dp(vi);
  end

  o = u.*u.*((kx*qx./(1+u*kx)).^2 + (ky*qy./(1+u*ky)).^2 + 1);

  end % newtonsolve()

  function ssd = exactparab(newton)
  % sum squared distance using exact method for paraboloids

  d2 = zeros(nd,1); % dist squared
  vi = true(nd,1); % bool index of valid (non-degenerate) samples
 
  % handle degenerate cases
  cp = abs(kx-ky)<eps; % circular paraboloid
  ypx = ~cp&&(abs(kx)<eps); % cylindric parab, x axis
  ypy = ~cp&&(abs(ky)<eps); % cylindric parab, y axis
  pl = ypx&&ypy; % plane
  if (cp||ypx||ypy); zz = z.*z; end
  if (~pl&&(cp||ypx)); yz = abs(y)<eps; end
  if (~pl&&(cp||ypy)); xz = abs(x)<eps; end
  if (pl) % plane
    d2 = zz; vi = false(nd,1);
  elseif (cp) % circular paraboloid
    ii = xz&yz; % on z axis, center of osculating sphere at origin: [0 0 2/k]
    jj = ((sign(z)~=sign(kx))|(z < 2/kx)); % "inside the cup" and beyond ctr?
    d2(ii&jj) = zz(ii&jj); % yup, closest point is origin
    d2(ii&~jj) = abs(2*z(ii)/kx); % nope, inf closest points on circle at z
    vi = vi&~ii;
  elseif (ypx) % cylindric paraboloid, x axis
    ii = yz&(abs(z-1/ky)<eps); % on zx plane & at ctr of osc sphere at origin
    d2(ii) = zz(ii); vi = vi&~ii;
  elseif (ypy) % cylindric paraboloid, y axis
    ii = xz&(abs(z-1/kx)<eps); % on zy plane & at ctr of osc sphere at origin
    d2(ii) = zz(ii); vi = vi&~ii;
  end

  % solve polynomial for non-degenerate points
  qx = x(vi); qy = y(vi); qz = z(vi); nv = length(x);
  qxx = qx.*qx; qyy = qy.*qy;

  % polynomial coefficients
  g = -4*(kxx*ky+kx*kyy); h = -2*(kxx+4*kxy+kyy);
  aa = -2*kxx*kyy*ones(nv,1);
  bb = aa.*qz + g;
  cc = g*qz + h;
  dd = h*qz + kxy*(ky*qxx+kx*qyy) - 4*(kx+ky);
  ee = 2*kxy*(qxx+qyy) - 4*(kx+ky)*qz - 2;
  ff = kx*qxx + ky*qyy - 2*qz;

  if (newton>0)
    dz = qz-p.zl(qx,qy); % initial guess based on 'vert' approx
    d2(vi) = min(newtonsolve(aa,bb,cc,dd,ee,ff,qx,qy,dz),...
                 newtonsolve(aa,bb,cc,dd,ee,ff,qx,qy,-dz));
  else
    for i = 1:nv
      d2(i)=polysolve([aa(i) bb(i) cc(i) dd(i) ee(i) ff(i)],qx(i),qy(i),qz(i));
    end
  end
  
  ssd = sum(d2);

  end % exactparab()

  function o = taubin1parab(varargin)
  % sum squared distance using first order Taubin approximation for paraboloids
  % see Taubin 1993
  c = abs(kx*x.^2 + ky*y.^2 - 2*z); ii = (c>eps); c = c(ii);
  b = 4*(kxx*x(ii).^2 + kyy*y(ii).^2 + 1);
  if ((nargin>0)&&varargin{1}) o = c./sqrt(b);
  else o = sum(c.*c./b); end
  end

  function o = taubin2parab(varargin)
  % sum squared distance using second order Taubin approximation for paraboloids
  % see Taubin 1993
  c = abs(kx*x.^2 + ky*y.^2 - 2*z); ii = (c>eps); c = c(ii);
  b = -2*sqrt(kxx*x(ii).^2 + kyy*y(ii).^2 + 1);
  a = -sqrt(kxx + kyy)*ones(length(ii),1);
  [s1, s2] = quadsol(a, b, c); sols = [s1, s2];
  sols((sols<0)|(imag(sols)~=0)) = nan;
  sols = nanmin(sols,[],2);
  if ((nargin>0)&&varargin{1}) o = sols;
  else o = nansum(sols.^2); end
  end

  function o = vertparab()
  o = sum((z-p.zl(x,y)).^2);
  end

% calculate residual %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

switch (p.s)
  
  case {'e','h','y','o'}; % paraboloid

    switch (type)
      case 'exact'; ssd = exactparab(0);
      case 'newton'; ssd = exactparab(1);
      case 'vert'; ssd = vertparab();
      case 'taubin1'; ssd = taubin1parab();
      case 'taubin2'; ssd = taubin2parab();
      otherwise; error('unknown residual type %s',type);
    end
   
  case 'p'; ssd = sum(z.*z); % plane
    
  case 's'; % sphere ctr=[0 0 1/k] rad=1/|k| (may be inf), k = kx = ky
       
    if (abs(kx) < tol); ssd = sum(z.^2);
    else ssd = sum((sqrt(x.*x + y.*y + (z-(1/kx)).^2)-abs(1/kx)).^2);
    end
    
  case 'c'; % circ cyl ctr=[0 0 1/ky] rad=1/|ky| (may be inf), axis=[1 0 0]
   
    if (abs(ky) < tol); ssd = sum(z.*z);
    else ssd = sum((sqrt(y.*y + (z-(1/ky)).^2)-abs(1/ky)).^2);
    end

  otherwise; error('unrecognized surface type %s',st);

end % switch (p.s)

res = sqrt(ssd/nd); % RMS

t = toc(tstart); if (nargout>1); sec = t; end

end

