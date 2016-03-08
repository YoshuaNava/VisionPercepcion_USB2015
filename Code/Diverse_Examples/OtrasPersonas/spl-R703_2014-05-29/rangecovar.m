function [cxx,cyy,czz,cxy,cyz,cxz] = rangecovar(mx,my,mz,r,model,k)
% rangecovar() apply uncertainty model to estimate range data covariances
%
%   The arguments mx, my, mz, and r must all be numeric arrays of the same
%   size MxN. (mx,my,mz) are the coordinates of the measurement ray
%   direction vectors *in camera frame* (z axis pointing out of camera, y
%   up, x left) and r the ranges of the data points along those vectors.
%
%   The outputs are MxN matrices of the sample point coordinate variances
%   and covariances of the 3D sample points x=mx.*r, y=my.*r, z=mz.*r. Note
%   that the "physical units" of the sample space are thus determined by
%   the scales of the measurement vector coordinates.
%
%   The argument k is a vector of one or more parameters for the error
%   model.  If omitted, it is set to a default depending on the model.
%
%   The argument model defaults to 'constant' if omitted. Otherwise it must
%   be one of the following:
%
%   'constant' - the covariance matrix for a sample with measurement vector
%   m is k(1)*m*m': constant uncertainty in range, independent of the
%   sample range, and no uncertainty in pointing direction. k(1) must be
%   nonnegative.
%
%   'linear' - the covariance matrix for a sample with range s and
%   measurement vector m is k(1)*s*m*m': uncertainty in range that scales
%   linearly with the sample range, and no uncertainty in pointing
%   direction. k(1) must be nonnegative.
%
%   'quadratic' - the covariance matrix for a sample with range s and
%   measurement vector m is k(1)*s^2*m*m': uncertainty in range that scales
%   quadratically with the sample range, and no uncertainty in pointing
%   direction. k(1) must be nonnegative.
%
%   'stereo' - applies Murray and Little's two-parameter error model for
%   stereo disparity. k must be a vector of five nonnegative parameters:
%   k = [p, m, b, f]; p is the variance of the pointing error of the
%   measurement vectors, represented as the variance in pixels of their
%   intersections with the image plane at z=f; m is the variance in the
%   disparity matching error, also measured in pixels; b is the stereo
%   baseline in physical units; f is the focal length in pixels. The model
%   is applied as if by (i) converting the given data to an xyz point cloud
%   in physical units (x=mx.*r, y=my.*r, z=mz.*r); (ii) backprojecting each
%   3D point to a corresponding 2D (u,v) pixel and disparity d (u=f*x./z,
%   v=f*y./z, d = f*b./z); (iii) calculating the covariance matrix for each
%   3D point in physical units as J*E*J', where E = diag([p, p, m]) and
%   J=[b/d, 0, -b*u./d.^2; 0, b/d, -b*v./d.^2; 0, 0, -f*b/d.^2].
%
%   If the argument k is omitted for the 'constant', 'linear', or
%   'quadratic' models, it defaults to [0.001*l] where l is the average
%   magnitude of the measurement vectors. If it is omitted for the 'stereo'
%   model it defaults to [0.01, 0.01, l, 320/tan(pi/6)].
%
%   BB2: 640x480 f=3.8mm=510px h,v=64,50d b=12cm, p=0.0025px^2, m=0.01px^2
%
%   kinect: 640x480 f=580px h,v=58,45d b=7.5cm, p=0.12px^2, m=0.030px^2
%
% Copyright (C) 2013 Marsette A. Vona

sz = size(r); % original input dimensions
n = sz(1)*sz(2); % number of points

if (any(size(mx)~=sz)||any(size(my)~=sz)||any(size(mz)~=sz))
  error('r, mx, my, and mz must all be same size');
end

% reshape inputs to column vectors
r = r(:); mx = mx(:); my = my(:); mz = mz(:);

% apply arg defaults
if ((nargin < 6)||isempty(k)); k = []; end
if ((nargin < 5)||isempty(model)); model = 'constant'; end

powmodels = {'constant', 'linear', 'quadratic'};

% compute moments if we'll need them
if (isempty(k)||any(strcmpi(model,powmodels)))
  mxx = mx.*mx; myy = my.*my; mzz = mz.*mz;
  mxy = mx.*my; myz = my.*mz; mxz = mx.*mz;
  if (isempty(k)); l = sum(sqrt(mxx+myy+mzz))/n; end
end

switch (model)
  case powmodels;
    if (isempty(k)); k = 0.001*l;
    elseif (k(1) < 0); error('k(1) must be nonnegative'); end
    kk = k(1)*ones(n,1);
    if (strcmpi(model,'linear')); kk = kk.*r;
    elseif (strcmpi(model,'quadratic')); kk = kk.*r.*r; end
    cxx = kk.*mxx; cyy = kk.*myy; czz = kk.*mzz;
    cxy = kk.*mxy; cyz = kk.*myz; cxz = kk.*mxz;
  case 'stereo';
    if (isempty(k)); k = [0.01, 0.01, l, 320/tan(pi/6)];
    elseif (any(k<0)); error('parameters must be nonnegative'); end
    p = k(1); m = k(2); b = k(3); f = k(4);
    x = mx.*r; y = my.*r; z = mz.*r;
    %    u = f*x./z; v = f*y./z; d = f*b./z;
    %    d2 = d.*d; pb2d2 = (p*b*b)./d2; mb2d4 = m*b*b./(d2.*d2);
    %    umb2d4 = u.*mb2d4; vmb2d4 = v.*mb2d4;
    %    cxx = pb2d2+u.*umb2d4; cxy = v.*umb2d4; cxz = f*umb2d4;
    %    cyy = pb2d2+v.*vmb2d4; cyz = f*vmb2d4; czz = f*f*mb2d4;
    xx = x.*x; yy = y.*y; zz = z.*z; xy = x.*y; xz = x.*z; yz = y.*z;
    mb2 = m/(b*b); zzf2 = zz/(f*f);
    cxx = zzf2.*(p+mb2*xx); cyy = zzf2.*(p+mb2*yy); czz = zzf2.*(mb2*zz);
    cxy = zzf2.*(mb2*xy); cyz = zzf2.*(mb2*yz); cxz = zzf2.*(mb2*xz);
  otherwise; error('unrecognized model %s',model);
end

% make outputs same shape as inputs
cxx = reshape(cxx,sz); cyy = reshape(cyy,sz); czz = reshape(czz,sz);
cxy = reshape(cxy,sz); cyz = reshape(cyz,sz); cxz = reshape(cxz,sz);

end
