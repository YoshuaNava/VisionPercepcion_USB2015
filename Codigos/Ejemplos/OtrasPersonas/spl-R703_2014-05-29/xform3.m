function [x, y, z] = xform3(x, y, z, r, t, invert, rotonly)
% xform3(x, y, z, r, t) applies the rigid body transform (r,t) to 3D points
%
%   The arguments x, y, z are the coordinates of the input points.  They must be
%   numeric matrices of the same size.
%
%   The outputs are matrices of the same size as the inputs containing the
%   coordinates of the transformed points.
%
%   t is the transform translation and must be 3x1 or 1x3.  r is the transform
%   rotation and may be 1x3, 3x1, or 3x3.  In the first two cases, rexp() is
%   applied to convert it to a 3x3 rotation matrix.
%
%   xform3(x,y,z,r,t,invert) optionally inverts the transform as it is applied.
%
%   xform3(x,y,z,r,t,invert,rotonly) optionally applies only the rotation.
%
% Copyright (C) 2013 Marsette A. Vona

if (nargin < 7); rotonly = 0; end
if (nargin < 6); invert = 0; end

if (isvector(r)); r = rexp(r); end

sz = size(x);

% one point per column
xyz = [x(:)'; y(:)'; z(:)'];

if (~rotonly)
  tt = repmat(t(:),1,sz(1)*sz(2));
  if (~invert); xyz = r*xyz+tt;
  else xyz = r'*(xyz-tt); end
else
  if (~invert); xyz = r*xyz;
  else xyz = r'*xyz; end;
end

% make outputs same shape as inputs
x = reshape(xyz(1,:),sz); y = reshape(xyz(2,:),sz); z = reshape(xyz(3,:),sz);

end
