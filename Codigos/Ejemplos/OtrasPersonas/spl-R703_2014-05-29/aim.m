function [x, y, z] = aim(p,u,ydir)
% [x, y, z] = aim(p,u) aims a 3D basis
%
%   The argument p is a vector giving the desired pointing direction of the z
%   axis of the returned basis.  Its length will be ignored, but it cannot be
%   zero.
%
%   The optional argument u is the desired "up" direction.  It should not be
%   parallel to p.  If u is omitted it defaults to [0,0,1]', unless that is
%   parallel to the given p, in which case [0,1,0]' is used.
%
%   Both p and u may be either 3x1 or 1x3.  The output vectors are always
%   3x1.
%
%   Either 1 or 3 outputs may be requested.  In the first case the return is
%   a 3x3 matrix [x,y,z].
%
%   The optional argument ydir must e either 'ydown' (the default), 'yup', or
%   a scalar.  In the latter case negative is synonymous with 'yup' and
%   nonnegative with 'ydown'.
%
%   The z axis of the returned basis is parallel to p. The x axis is parallel to
%   cross(p,u) if ydir is 'ydown', or cross(u,p) if ydir is 'yup'.  The y axis
%   is cross(z,x).  Hence z always points into the scene from the camera, for
%   'ydown' x points right in camera frame and y points down, and for 'yup' x
%   points left and y points up.
%
% Copyright (C) 2013 Marsette A. Vona

p = p(:)/norm(p);

xdir = 1;
if (nargin>=3);
  if (ischar(ydir))
    switch (ydir)
      case 'ydown'; xdir = 1; case 'yup'; xdir = -1;
      otherwise; error('unrecognized ydir %s',ydir);
    end
  elseif (ydir<0); xdir = -1; end
end

if ((nargin<2)||isempty(u))
  l = cross(p,[0;0;1]);
  if (sum(l.*l)<eps); l = cross(p,[0;1;0]); end
else l = cross(p,u(:)); end

l = l/norm(l);

z = p; x = l*xdir; y = cross(z,x);

if (nargout<=1); x = [x,y,z]; end

end
