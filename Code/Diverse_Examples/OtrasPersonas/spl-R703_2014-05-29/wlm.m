function [a, r, E, V] = wlm(f,d,S,a0,varargin)
% a = wlm(f,d,S,a0) chi^2 optimization by weighted Levenberg-Marquardt
%
%   Argument f is a vector function to optimize.  It must take two inputs, the
%   first of size NDxDD, and the second of size NAx1 or 1xNA.  ND is the number
%   of datums, DD is the data dimension, and NA is he number of parameters.
%
%   Argument d is the (constant) NDxDD data matrix.
%
%   Argument S is either empty or DDxDDxND.  Each plane must be a positive
%   semi-definite covariance matrix for the corresponding datum.  If supplied
%   as empty every covariance matrix is treated as identity.
%
%   Argument a0 is an NAx1 or 1xNA vector giving the initial parameter
%   estimate.
%
%   wlm() acts to find a that minimizes norm(f(d,a)) by LM, with each datum
%   weighted by the inverse stdev of f(d,a) relative to that datum's
%   covariance matrix.
%
%   wlm(...) can produce from one to four outputs.  a are the fitted parameters.
%   r is the final residual.  E and V have the same meaning as for lm(), which
%   see.
%
%   OPTIONS
%
%   A list of name,value pairs may be given to set options.  Unrecognized names
%   cause warnings.  If a name is given more than once the last-given
%   (rightmost) value takes precedence.
%
%   All of the options that lm() accepts are also accepted here.  They are
%   passed on in the final call to lm(), and where appropriate, they also
%   apply to the preparatory computations.
%
%   Option 'unweighted' (default 0): disables weighting, effectively making
%   wlm() just a cover of lm().
%
%   Option 'dfdd' (default 'num'): j = dfdd(a) calculates j as the (nd x dd)
%   Jacobian of f(d,a) with respect to d. numj() is used if dfdd is 'num' or [].
%
%   Option 'dfda' (default 'num'): j = dfda(a) calculates j as the (nd x na)
%   Jacobian of f(d,a) with respect to a.  numj() is used if dfda is 'num' or
%   [].
%
%   Option 'd2fdadd' (default 'num'): h = d2fdadd(a) calculates h as the (dd x
%   na x nd) Hessian of f(d,a) first with respect to d, then a.  numj() is used
%   if d2fdadd is 'num' or [].
%
%   Option 'numdwfda' (default 0): whether to calculate the Jacobian of the
%   weighted error function numerically with numj().  Otherwise it is
%   calculated analytically from the other functions.
%
%   Option 'chknd' (default 0): if positive, and numdwfda was 0, then compute
%   every dwfda both analytically and by numj() and compare them.  Raise an
%   error if any entry differs by more than chknd.
%
%   Option 'minvar' (default eps): minimum clamp for calculated variances
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
unweighted = 0; numdwfda = 0;
dfdd = []; dfda = []; d2fdadd = [];
inplay = [];
minvar = eps;  chknd = 0;
dbg = 0;

ki = true(nva,1);
for i=1:nopt
  iv = 2*i; in = iv-1;
  n = varargin{in}; v = varargin{iv};
  if (ischar(n))
    switch (n)
      case 'unweighted'; unweighted = v; ki(in:iv) = false;
      case 'numdwfda'; numdwfda = v; ki(in:iv) = false;
      case 'dfdd'; dfdd = v; ki(in:iv) = false;
      case 'dfda'; dfda = v; ki(in:iv) = false;
      case 'd2fdadd'; d2fdadd = v; ki(in:iv) = false;
      case 'minvar'; minvar = eps; ki(in:iv) = false;
      case 'inplay'; inplay = v; % passthrough
      case 'chknd'; chknd = v; case 'dbg'; dbg = v; % passthrough
      %otherwise; warning('unexpected optional arg %s',n); % passthrough 
    end
  else warning('non-string, expected name'); end
end

lmopts = {varargin{ki}};

sz = size(d);
nd = sz(1); % num datums
dd = sz(2); % data dimension
assert(isvector(a0));
na = length(a0); % num params
isndfdd = (isempty(dfdd)||(ischar(dfdd)&&strcmpi(dfdd,'num')));
isndfda = (isempty(dfda)||(ischar(dfda)&&strcmpi(dfda,'num')));
isnd2fdadd = (isempty(d2fdadd)||(ischar(d2fdadd)&&strcmpi(d2fdadd,'num')));

if (isempty(S)); S = repmat(eye(dd,dd),[1,1,nd]); end

if (~isempty(inplay))
  assert(isvector(inplay)&&islogical(inplay)&&(length(inplay)==na));
else inplay = true(na,1); end

% (nd x 1) error as a function of a
  function e = fa(a); e = f(d,a); end

% (nd x dd) Jacobian of f(d,a) with respect to d
if (isndfdd); ndd = sqrt(eps(max(d))); nddip = true(1,dd); end
  function j = ndfdd(a); j = numj(f,d,ndd,nddip,a); end
if (isndfdd); dfdd = @ndfdd; end

% (nd x na) Jacobian of f(d,a) with respect to a
  function j = ndfda(a); j = numj(@fa,a,[],inplay); end
if (isndfda); dfda = @ndfda; end

% (dd x na x nd) Hessians of f(d,a), first with respect to d, then a
  function j = ndfddt(a); j = reshape(dfdd(a)',dd,1,nd); end
  function h = nd2fdadd(a); h = numj(@ndfddt,a,[],inplay); end
if (isnd2fdadd); d2fdadd = @nd2fdadd; end

% (nd x 1) variance of f(d,a) at a
amo = []; vmo = []; % memo
  function v = fvar(a)

  if (isequal(a,amo)); v = vmo; return; end

  v = zeros(nd,1); j = dfdd(a);

  %  v = zeros(nd,1);
  %  for i=1:nd
  %  v(i) = j(i,:)*S(:,:,i)*j(i,:)';
  %  if (v(i)<0); error('S for datum %d not positive semi-definite',i); end
  %  end

  % much faster
  Sj = reshape(sum(reshape(reshape(S,dd,dd*nd).*...
                           repmat(reshape(j',1,dd*nd),dd,1),...
                           dd,dd,nd),...
                   2),...
               dd,nd);
  v = sum(j.*Sj',2);
  ei = find(v<0,1);
  if (~isempty(ei)); error('S for datum %d not positive semi-definite',ei); end

  vi = (v<minvar); if (all(vi)); v(:) = 1; else v(vi) = minvar; end

  amo = a; vmo = v; 

  end

% (nd x 1) weighted error of f(d,a) at a
  function e = wf(a)
  e = f(d,a); if (~unweighted); e = e./sqrt(fvar(a)); end
  end

% (nd x na) analytic Jacobian of wf with respect to a
  function j = adwfda(a)
  ja = dfda(a);
  if (unweighted); j = ja;
  else
    e = f(d,a); v = fvar(a); jd = dfdd(a); H = d2fdadd(a);

    %j = zeros(nd,na);
    %for i=1:nd
    %  j(i,:) = (ja(i,:)-(e(i)/v(i))*(jd(i,:)*S(:,:,i)*H(:,:,i)))/sqrt(v(i));
    %end

    % much faster
    jdS = reshape(sum(reshape(reshape(S,dd,dd*nd).*...
                              repmat(reshape(jd',1,dd*nd),dd,1),...
                              dd,dd,nd),...
                      2),...
                  dd,nd)';
    jdSH = reshape(sum(repmat(reshape(jdS',dd,1,nd),1,na).*H),na,nd)';
    j = (ja-jdSH.*repmat(e./v,1,na)).*repmat(1./sqrt(v),1,na);

    j(:,~inplay) = 0;

  end
  if (chknd>0); chkj(j,numj(@wf,a,[],inplay),chknd,'dwfda'); end
  end

% (nd x na) numeric Jacobian of wf with respect to a
  function j = ndwfda(a); j = numj(@wf,a,[],inplay); end

dwfda = @adwfda;
if (numdwfda); dwfda = @ndwfda; end

  function s = dtype(isnd); if (isnd); s = 'num'; else s = 'analytic'; end; end

if (dbg)
  fprintf('wlm: unweighted=%d, numdwfda=%d, chknd=%d\n',...
          unweighted,numdwfda,chknd);
  fprintf('wlm: dfdd=%s, dfda=%s, d2fdadd=%s\n',...
          dtype(isndfdd),dtype(isndfda),dtype(isnd2fdadd));
end

switch(nargout)
  case 4; [a,r,E,V] = lm(dwfda,@wf,a0,lmopts{:});
  case 3;   [a,r,E] = lm(dwfda,@wf,a0,lmopts{:});
  case 2;     [a,r] = lm(dwfda,@wf,a0,lmopts{:});
  otherwise;    [a] = lm(dwfda,@wf,a0,lmopts{:});
end

if (dbg); fprintf('wlm: %gs\n',toc(tstart)); end

end

function chkj(j,nj,tol,n)
[r,c] = find(abs(j-nj)>tol,1);
if (~isempty(r))
  %  j, nj
  error('abs(%s-num%s)=%g>%g at (%d,%d)',n,n,abs(j(r,c)-nj(r,c)),tol,r,c);
end
end
