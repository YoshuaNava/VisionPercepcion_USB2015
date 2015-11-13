function jj = numj(e,a,nda,inplay,varargin)
% jj = numj(e,a) calculates numeric Jacobian of e(a) at a
%
%   Input e is the handle of a function which produces an Mx1xK output vector
%   given input a, which may be 1xN, Nx1, or LxN.  e() will be called N+1 times.
%
%   Output jj is an estimate of the MxNxK Jacobian of e at a calculated using
%   the two-point secant method.
%
%   Example: numj(@(x)([sin(x),cos(x)]'),pi/2) yields [0,-1]' since the
%   derivative of sin is cos, the derivative of cos is -sin, and [cos(pi/2),
%   -sin(pi/2)]'=[0,-1]'.
%
%   The optional parameter nda gives the numerical differential added to each
%   element (column) of a to calculate the derivative.  It may be either a
%   scalar, an Nx1 vector, or empty (the default), which automatically
%   calculates nda = sqrt(eps(max(a,1))) if a is a vector or
%   nda=sqrt(eps(max(a,1))) if a is a matrix.
%
%   The optional parameter inplay is either empty or an Nx1 or 1xN vector of
%   logicals specifying which of the parameters are "in play".  Jacobian
%   columns corresponding to parameters not in play will be set to zero.
%   If inplay is empty (the default), then all parameters are in play.
%
%   Any additional parameters are passed on as extra arguments to e.
%
% Copyright (C) 2013 Marsette A. Vona

a0 = a;

if (isvector(a)); l = 0; n = length(a); else [l,n] = size(a); end

e0 = e(a0,varargin{:}); [m,en,k] = size(e0);

assert(en==1);

if ((nargin < 3)||isempty(nda))
  if (l==0); nda = sqrt(eps(max(a,1))); else nda = sqrt(eps(max(a,1))); end
end
if (isscalar(nda)); nda = repmat(nda,n,1); end
assert(isvector(nda)&&(length(nda)==n));

if ((nargin < 4)||isempty(inplay)); inplay = true(n,1); end
assert(isvector(inplay)&&islogical(inplay)&&(length(inplay)==n));

jj = zeros(m,n,k);

for i=1:n
  if (inplay(i))
    d = nda(i); if (l==0); a(i) = a(i)+d; else a(:,i) = a(:,i)+d; end
    jj(:,i,:) = (e(a,varargin{:})-e0)/d;
    if (l==0); a(i) = a0(i); else a(:,i) = a0(:,i); end
  end
end

end
