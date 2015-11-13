function [o,t0,t1,flip] = rreparam(r,near)
%reparam(r) maps orientation vector r to equivalent with length at most pi
%
%  There is still a choice of two possible outputs when
%  norm(rreparam(r))~=pi.  This is resolved by a sign policy.
%
%  Optional input near is another arbitrary orientation vector, not
%  necessarily canonical (i.e. no constraints on norm(near)).  The returned
%  vector will be selected as the alias of r nearest near.  This overrides
%  both the default behavior of returning a canonical r with norm(r)<=pi, and
%  also the sign policy when norm(r)~=pi.
%
% Copyright (C) 2013 Marsette A. Vona

% map near down to at most 2pi, but remember its offset as nk*2*pi.
if (nargin>1)
  tn = norm(near);
  nk = floor(tn/(2*pi));
  near = near*((tn-nk*2*pi)/tn);
end

% always map r to at most pi
t = norm(r); t0 = t; nt = t;
k = floor(nt/(2*pi));
if (k~=0) nt = nt-k*2*pi; end
if (nt>pi); nt = nt-2*pi; end % negative
if (nt~=t); r = r*(nt/t); t = -nt; end;

flip = 0;

if (nargin>1)
  altt = t-2*pi; % negative
  altr = r*(altt/t);
  dr = r-near; altdr = altr-near;
  if (sum(altdr.*altdr)<sum(dr.*dr)); r = altr; t = -altt; end
  if (nk~=0); r = (1+nk*2*pi)*(r/t); end % restore original offset of near
else % apply sign policy to resolve ambiguity when t~=pi
  if (abs(abs(t)-pi)<100*sqrt(eps))
    if (r(1)<0); r = -r; flip = 1;
    elseif (r(1)==0) % on circle in yz plane
      if (r(2)<0); r = -r; flip = 1;
      elseif (r(2)==0) % on z axis
        if (r(3)<0); r = -r; flip = 1; end
      end
    end
  end
end

t1 = t;

o = r;

end
