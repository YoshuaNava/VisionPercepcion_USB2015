function [a, E, V] = lls(J,b,inplay)
% a = lls(J,b) solves the system J*a=b by linear least squares
%
%   J is an MxN matrix.  b must be an Mx1 column vector, or, if empty or
%   omitted, it defaults to zeros(M,1) and the algorithm will solve for a
%   homogenous solution with norm(a)=1 (there is a corner case involving the
%   optional inplay parameter, see below).
%
%   The algorithm uses svd() and sets singular values less than an appropriate
%   tolerance to zero.
%
%   [a, E, V] = lls(...) can produce from one to three outputs.  Output a is
%   always an Nx1 column vector containing the solution.  When producing two
%   outputs, the second output E is an NxN positive semi-definite covariance
%   matrix for the solution a.  When producing three outputs, E is an Nx1 vector
%   of the eigenvalues of the covariance matrix sorted from smallest to largest,
%   and V is an NxN orthonormal matrix of the corresponding eigenvectors such
%   that the covariance matrix is V*diag(E)*V'.
%
%   The eigenvalues of the covariance matrix are calculated as the squares of
%   either the inverses or the pseudoinverses of the singular values of J; the
%   inverses are used when returning the covariance in factored E, V format, and
%   the pseudoinverses are used when returning just the covariance matrix E.  As
%   long as the entries in J were finite no eigenvalue will be zero in either
%   case (a zero eigenvalue in a covariance matrix indicates no uncertainty in
%   the direction of the corresponding eigenvector).  However, when J is not
%   full rank the covariance matrix will (correctly) have infinite eigenvalues;
%   the corresponding eigenvectors span the nullspace of J.  However, infinite
%   eigenvalues can only be represented correctly by keeping the covariance
%   matrix in factored form.
%
%   The optional parameter inplay is either empty or an Nx1 or 1xN vector of
%   logicals specifying which of the parameters are "in play".  Parameters not
%   in play will be set to zero, along with the corresponding enries in E.  If
%   inplay is empty (the default), then all parameters are in play.  Setting all
%   entries in inplay false while solving a homogenous problem triggers a corner
%   case---the solution in that case will be any Nx1 unit vector.
%
% Copyright (C) 2013 Marsette A. Vona

% TBD write test based on the verification that the residual vector b-Ja
% should be orthogonal to the column space of J, i.e. that norm((b-Ja)'*J)
% should be zero

tstart = tic();

dbg = 0;

[m, n] = size(J);

if ((nargin < 3)||isempty(inplay)); inplay = true(n,1); end
assert(isvector(inplay)&&islogical(inplay)&&(length(inplay)==n));

if ((nargin < 2)||isempty(b)); b = zeros(m,1); end

J(:,~inplay)=0; % out-of-play parameter subspace should be in the nullspace of J

% compute svd J = U*S*V'
[~,S,V] = svd(J,0); % singular values come out sorted in decreasing order

% in full svd, U is MxM, S is MxN, and V is NxN
%
% in *this version* of the econ svd, the above holds if M<=N
%
% otherwise U is MxN, S is NxN, and V is (still) NxN

% kill singular values that are below tol
sv = diag(S);
assert(length(sv)==min(m,n));
if (m < n); sv = [sv; zeros(n-m,1)]; end % force sv to Nx1
tol = max(size(J))*eps(max(sv)); % see rank()
sv(sv<tol) = 0;

isv = 1./sv; % diagonal of S^{-1}, i.e. inverse singular values, may be infinite
pv = isv; pv(isinf(pv)) = 0; % diagonal of the pseudoinverse of S

%          J'*J*a=J'*b
% V*S*U'*U*S*V'*a=J'*b
%      V*S*S*V'*a=J'*b
%               a=V*P*P*V'*J'*b

if (any(b)); a = V*diag(pv.*pv)*V'*J'*b;
else % homogenous solution
  % try to find last col of V that is wholly within the inplay subspace
  % note: all([]==0) = true
  for i=n:-1:1; a = V(:,i); if (all(a(~inplay)==0)); break; end; end
end

% calculate output covariance matrix (V is already set)
if (nargout == 3); E = isv.*isv; E(~inplay) = 0;
elseif (nargout == 2); E = V*diag(pv.*pv)*V'; end

if (dbg); fprintf('lls: %gs\n',toc(tstart)); end

end

