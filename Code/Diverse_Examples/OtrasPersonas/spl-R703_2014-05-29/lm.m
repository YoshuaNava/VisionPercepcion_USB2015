function [a, r, E, V] = lm(J,e,a0,varargin)
% a = lm(J,e,a0) locally minimizes norm(e(a)) by Levenberg-Marquardt
%
%   e is a function which produces an Mx1 error vector given Nx1 parameter
%   vector a.  J is a function which produces the MxN Jacobian of e at a or []
%   or 'num' to compute J numerically from e with numj().  a0 is an initial
%   estimate for a.
%
%   LM seeks to minimize the residual r = norm(e(a)) by iterative local
%   linearization starting at a0.
%
%   lm(...) can produce from one to four outputs.  a are the fitted parameters.
%   r is the final residual.  E and V have the same meaning as for lls(), which
%   see.
%
%   Example: [a, r] = lm('num',@(x)((x-3)*(x-4)),5) returns a=4, r=0
%
%   Example: [a, r] = lm(@(x)(2*x-7),@(x)((x-3)*(x-4)),5,'dbg',1) also returns
%   a=4, r=0, but is generally faster than using numeric Jacobian (after JIT)
%
%   OPTIONS
%
%   A list of name,value pairs may be given to set options.  Unrecognized names
%   cause warnings.  If a name is given more than once the last-given
%   (rightmost) value takes precedence.
%
%   Option 'inplay' (default []): see lls().  Implies svdalways=1.
%
%   Option 'maxi' (default 100): if nonnegative, sets a maximum limit on the
%   number of iterations.  May be zero to just compute outputs for unmodified
%   parameter vector.
%
%   Option 'minad' (default 0): if positive, iterations terminate whenever the
%   absolute decrease in residual is smaller than minad.
%
%   Option 'minrd' (default 0): if positive, iterations terminate whenever the
%   decrease in residual is less than minrd times the previous residual.
%
%   Option 'minar' (default 0): if positive, iterations terminate whenever
%   the residual is less than minar.
%
%   Option 'minrr' (default 0): if positive, iterations terminate whenever the
%   residual is less than minrr times its original value norm(e(a0)).
%
%   Option 'minada' (default 0): if positive, iterations terminate whenever
%   the magnitude of the parameter differential da is less than minada.
%
%   Option 'minrda' (default 1e-6): if positive, iterations terminate whenever
%   the magnitude of the parameter differential da is less than minrda times the
%   magnitude of the parameter vector a.
%
%   Option 'l0' (default 0.001): initial damping factor, must be positive.
%
%   Option 'nu' (default 10): damping factor multiplier, must be greater than
%   one.
%
%   Option 'fda' (default []): if non-empty, this is a function called after
%   every parameter vector adjustment, passing the adjustment da.  It must
%   return at least one output; if that is non-empty it is saved back to da
%   before it is added to the parameter vector.
%
%   Option 'fa' (default []): if non-empty, this is a function called on the
%   parameter vector after it has been adjusted on every iteration, but before
%   checking for divergence.  It must return at least one output; if that is
%   non-empty it is saved back to a.
%
%   Option 'upd' (default []): if non-empty, this is a function called on the
%   parameter vector, an int, and a boolean after every iteration, and before
%   the first.  The int is the iteration number.  The boolean is true if the
%   iteration resulted in a committed update, and false otherwise.
%
%   Option 'normalize' (default 0): whether to normalize the parameter vector
%   after every iteration, and before return.
%
%   Option 'svdalways' (default 1): whether to use svd instead of backslash
%   (mldivide) to solve the local system formed at each iteration (a final svd
%   is always required when computing the optional outputs E and V).
%
%   Option 'nda' (default []): parameter stepsize for numeric derivatives,
%   see numj() for details.
%
%   Option 'chknd' (default 0): if positive, and if J was not passed as 'num',
%   then compute every Jacobian both by the passed J function and by numj() and
%   compare them.  Raise an error if any entry differs by more than chknd.
%
%   Option 'dbg' (default 0): may have any nonnegative value.  dbg=0 disables
%   debug.  Larger values enable successively more verbose debug.
%   dbg = 1: non-iteration messages
%   dbg = 2: all messages
%   dbg = 3: all messages+pauses
%
% Copyright (C) 2013 Marsette A. Vona

tstart = tic();

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
maxi = 100;
minad = 0; minrd = 0; minar = 0; minrr = 0;
minada = 0; minrda = 1e-10;
l0 = 0.001; nu = 10;
fda = []; fa = []; upd = [];
inplay = [];
svdalways = 1; normalize = 0;
nda = []; chknd = 0;
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'maxi'; maxi = v;
      case 'minad'; minad = v; case 'minrd'; minrd = v;
      case 'minar'; minar = v; case 'minrr'; minrr = v;
      case 'minada'; minada = v; case 'minrda'; minrda = v;
      case 'l0'; l0 = v; case 'nu'; nu = v;
      case 'fda'; fda = v; case 'fa'; fa = v; case 'upd'; upd = v;
      case 'inplay'; inplay = v;
      case 'svdalways'; svdalways = v; case 'normalize'; normalize = v;
      case 'nda'; nda = v; case 'chknd'; chknd = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string, expected name'); end
end

if (l0 < 0); error('l0 must be positive'); end
if (nu <= 1); error('nu must be greater than 1'); end

if (dbg)
  fprintf('minad=%g, minrd=%g, minar=%g, minrr=%g, minada=%g, minrda=%g\n',...
          minad, minrd, minar, minrr, minada, minrda);
end

% set initial values
a = a0; l = l0;

[n, an] = size(a);
if (an~=1); error('a0 must be an Nx1 vector'); end

if (~isempty(inplay))
  svdalways = 1;
  assert(isvector(inplay)&&islogical(inplay)&&(length(inplay)==n));
else inplay = true(n,1); end

i = 0; % iteration
cc = false; % whether the last iteration committed a change to a

% calculate initial error and residual
if (normalize); a = a/norm(a); cc = true; end
ee = e(a); r = norm(ee); r0 = r;

[m, en] = size(ee);
if (en~=1); error('e(a) must be an Mx1 vector'); end

isnj = (isempty(J)||(ischar(J)&&strcmpi(J,'num')));
  function j = nj(a); j = numj(e,a,nda,inplay); end
if (isnj); J = @nj; chknd = 0; end

jj = []; E = []; V = []; % sentinels

while true
  
  if (~isempty(upd)); upd(a,i,cc); end
  
  if (dbg>1)
    fprintf('iteration %d: residual=%g, lambda=%g\n',i,r,l);
    %if (i>0)
    %  fprintf('residual r=%g, dr=%g, dr/lastr=%g, r/r0=%g\n',...
    %          r, dr, dr/lastr, r/r0);
    %  fprintf('parameter norm ||a||=%g, ||da||=%g, ||da||/||a||=%g\n',...
    %          norm(a), norm(da), norm(da)/norm(a));
    %end
  end
  
  if (dbg>2)
    es = input('lm eval (q <enter> quits, <enter> continues):','s');
    if (strcmpi(es,'q')); break; else eval(es); end
  end
  
  if (r==0)
    if (dbg); fprintf('terminating due to 0 residual at iteration %d\n',i); end
    break;
  end
  
  if ((i>0)&&(minad>0)&&(dr<0)&&(abs(dr)<minad))
    if (dbg); fprintf('terminating due to minad at iteration %d\n',i); end
    break;
  end
  
  if ((i>0)&&(minrd>0)&&(dr<0)&&(abs(dr)<minrd*lastr))
    if (dbg); fprintf('terminating due to minrd at iteration %d\n',i); end
    break;
  end
  
  if ((minar>0)&&(r<minar))
    if (dbg); fprintf('terminating due to minar at iteration %d\n',i); end
    break;
  end
  
  if ((minrr>0)&&(r<minrr*r0))
    if (dbg); fprintf('terminating due to minrr at iteration %d\n',i); end
    break;
  end
  
  if ((i>0)&&(minada>0)&&((da'*da)<(minada*minada)))
    if (dbg); fprintf('terminating due to minada at iteration %d\n',i); end
    break;
  end
  
  if ((i>0)&&(minrda>0)&&((da'*da)<(minrda*minrda*a'*a)))
    if (dbg); fprintf('terminating due to minrda at iteration %d\n',i); end
    break;
  end
  
  if (i==maxi)
    if (dbg); fprintf('terminating due to max iterations %d\n',maxi); end
    break;
  end
  
  i = i+1;
  
  % update Jacobian if necessary
  if ((i==1)||(dr<0)); jj = J(a); end
  if (chknd>0); chkj(e,a,nda,inplay,jj,chknd); end
  
  [mm, nn] = size(jj);
  assert ((mm==m)&&(nn==n),'J(a) must be MxN=%dx%d',m,n);

  % calculate da
  jtj = jj'*jj; aa = jtj+l*diag(diag(jtj)); b = -jj'*ee;
  
  if (svdalways); [da,E,V] = lls(aa,b,inplay); else da = aa\b; end
  
  if (~isempty(fda)); o = fda(da); if (~isempty(o)); da = o; end; end
  
  % update a (will back out the update if residual increased)
  lasta = a; a = a+da;
  if (~isempty(fa)); o = fa(a); if (~isempty(o)); a = o; end; end
  
  if (normalize); a = a/norm(a); end
  
  % update error and residual
  lastr = r; lastee = ee;
  ee = e(a); r = norm(ee);
  dr = r-lastr;
  
  if (dr<0) % converging
    l = l/nu; cc = true;
    if (dbg>1); fprintf('converging at iteration %d, updating a\n',i); end
  else % diverging
    l = l*nu; a = lasta; ee = lastee; r = lastr; cc = false;
    if (dbg>1); fprintf('diverging at iteration %d, not updating a\n',i); end
  end
  
end % main loop

% calculate output covariance matrix
if (nargout > 2)
  if (isempty(jj))
    jj = J(a); if (chknd>0); chkj(e,a,nda,inplay,jj,chknd); end; end
  if (isempty(E)); [~,E,V] = lls(jj); end
  if (nargout == 3); E(isinf(E)) = 0; E = V*diag(E)*V'; end
end

if (dbg)
  fprintf('final residual %g (change of %g) after %d iterations\n',r,r-r0,i);
end

if (normalize&&(i==0)); a = a/norm(a); end

if (dbg); fprintf('lm: %gs\n',toc(tstart)); end

end % lm

function o = chkj(e,a,nda,inplay,jj,tol)
% compare jj with numj(a,a,nda,inplay)
njj = numj(e,a,nda,inplay);
[r,c] = find(abs(jj-njj)>tol,1);
if (~isempty(r))
  error('abs(J-numj)=%g > %g at (%d,%d)',abs(jj(r,c)-njj(r,c)),tol,r,c);
end
end
