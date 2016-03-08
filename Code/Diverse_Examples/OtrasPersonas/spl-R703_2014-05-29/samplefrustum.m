function [mx, my, mz] = samplefrustum(hfov,vfov,nh,nv,p,u)
% samplefrustum(hfov,vfov,nh,nv) generates range sample measurement vectors
%
%   The outputs are three nh by nv arrays mx, my, mz with the sample vector
%   coordinates.  The vectors are arranged in a regular rectangular grid
%   about the pointing direction p, with the vector u giving the "up"
%   direction.  hfov and vfov give the horizontal and vertical fields of view
%   in radians.
%
%   samplefrustum(hfov,vfov,nh,nv,p,u) explicitly gives both p and u (their
%   lengths are ignored, but must be nonzero).
%
%   samplefrustum(hfov,vfov,nh,nv,p) uses the default u=[0,1,0]'.
%
%   samplefrustum(hfov,vfov,nh,nv) uses the default u and the default
%   p=[0,0,1]'.
%
% Copyright (C) 2013 Marsette A. Vona

if ((nargin < 6)||isempty(u)); u = [0 1 0]'; end
if ((nargin < 5)||isempty(p)); p = [0 0 1]'; end

if (any([hfov,vfov,nh,nv])<=0); error('inputs must be positive'); end

p = p/norm(p); u = u/norm(u); l = cross(u,p); % pointing, up, left

% don't do it this way, samples a spherical rectangle
%az = repmat(linspace(-hfov/2,hfov/2,nh),nv,1);
%el = repmat(linspace(-vfov/2,vfov/2,nv)',1,nh);
%[xx, yy, zz] = sph2cart(az,el,1); % (1,0,0) is at (az,el)=(0,0)

% sample like a camera would
ll = tan(hfov/2)*(nh-1)/nh; uu = tan(vfov/2)*(nv-1)/nv;
y = 0; if (nh>1); y = linspace(-ll,ll,nh); end
yy = repmat(y,nv,1);
z = 0; if (nv>1); z = linspace(-uu,uu,nv)'; end
zz = repmat(z,1,nh);
xx = ones(nv,nh);
nn = sqrt(xx.*xx+yy.*yy+zz.*zz);
xx = xx./nn; yy = yy./nn; zz = zz./nn;

rr = [p(:), l(:), u(:)]; % rotation matrix

xyz = rr*[xx(:)'; yy(:)'; zz(:)']; % rotate points

% extract coordinates
xx = xyz(1,:); yy = xyz(2,:); zz = xyz(3,:);
mx = reshape(xx,nv,nh); my = reshape(yy,nv,nh); mz = reshape(zz,nv,nh);

end % samplefrustum
