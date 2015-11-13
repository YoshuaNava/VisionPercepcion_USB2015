function varargout = patchsample(p,mx,my,mz,varargin)
% r = patchsample(p,mx,my,mz) computes range samples of a patch
%
%   The argument p is the patch to sample.
%
%   The arguments mx, my, mz must all be numeric arrays of the same size MxN.
%   They give the coordinates of the measurement ray direction vectors.  The
%   starting points default to (0,0,0) unless 'cx', 'cy', and 'cz' are
%   specified, see OPTIONS below.
%
%   All passed coordinates are assumed to be in the world frame of p unless the
%   optional argument 'frame' is used to specify local frame.
%
%   patchsample(...) may produce 1, 4, or 10 output arguments.  The first output
%   is always the range r, possibly perturbed by Gaussian noise using covariance
%   matrices from rangecovar() (see OPTIONS, below).  If 4 or 10 outputs are
%   requested, the second through fourth are always mx, my, mz, possibly
%   perturbed by noise.  If 10 outputs are requested, the fifth through tenth
%   are the output xyz covariances cxx,cyy,czz,cxy,cyz,cxz calculated by
%   rangecovar().
%
%   Upon return, the coordinates of the sample points are always cx+mx.*r,
%   cy+my.*r, cz+mz.*r.  Measurement rays which did not hit the patch surface
%   yield infinite range, unless clean=1, perturb=1, or covariance matrices were
%   requested, in which case failed intersections are discarded.  Note that the
%   "units" of r are the lengths of the measurement vectors.
%
%   The outputs are all either MxN, if failed intersections were not discarded,
%   and otherwise Gx1 where G is the actual number of good intersections.
%
%   OPTIONS
%
%   A list of name,value pairs may be given to set options.  Unrecognized names
%   cause warnings.  If a name is given more than once the last-given
%   (rightmost) value takes precedence.
%
%   Options 'cx', 'cy', 'cz' (default 0): coordinates of the measurement ray
%   starting points.  May each either be present or absent and scalar or MxN.
%
%   Option 'clip' (default 1): whether to suppress intersections whose
%   projection on the local frame XY plane falls outside the patch bounds.  They
%   will be set to inf unless clean=1, in which case they will be discarded
%   entirely.
%
%   Option 'nonnegative' (default 1): whether to suppress intersections with
%   negative range.  They will be set to inf unless clean=1, in which case they
%   will be discarded entirely.
%
%   Option 'clean' (default 0): whether to remove intersections with infinite
%   range.  This always happens after clip and nonnegative have been applied.
%
%   Option 'frame' (default 'world'): the coordinate frame of the supplied
%   measurement rays, must be either 'world' or 'local'.
%
%   Option 'errormodel' (default []): if non-empty, then this specifies the
%   error model to use for rangecovar(); otherwise rangecovar()'s default is
%   used.
%
%   Option 'errorparams' (default []): if non-empty, then this specifies the
%   error model parameters to use for rangecovar(); otherwise rangecovar()'s
%   default is used.
%
%   Option 'perturb' (default 0): whether to add Gaussian noise to the
%   samples using covariance matrices calculated by rangecovar().
%
%   Option 'fp' (default [0,0,1]): if nonempty this specifies the z-axis of
%   camera frame (the camera pointing vector).
%
%   Option 'fu' (default [0,1,0]): if nonempty this specifies the y-axis of
%   camera frame (the camera up vector).
%
%   Option 'fc' (default [0,0,0]): if nonempty this specifies the camera
%   frame center of projection.
%
%   Option 'dbg' (default 0): may have any nonnegative value.  dbg=0 disables
%   debug.  Larger values enable successively more verbose debug.
%
% Copyright (C) 2013 Marsette A. Vona

tstart = tic();

if ((nargout~=1)&&(nargout~=4)&&(nargout~=10))
  error('expected 1, 4, or 10 outputs');
end

om = 0; ocov = 0;
if (nargout>=4) om = 1; end
if (nargout==10) ocov = 1; end

% (re)generate patch surface and boundary fields
p = patchchk(p,'gf',1,'gb',1);

% parse arguments

sz = size(mx);

if (any(size(my)~=sz)||any(size(mz)~=sz))
  error('mx, my, and mz must all be same size');
end

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expected even number of varargs (name,value pairs), got %d',nva);
  nva = nva-1;
end
nopt = nva/2;

% option defaults
cx = 0; cy = 0; cz = 0;
clip = 1; nonnegative = 1; clean = 0;
frame = 'world';
errormodel = []; errorparams = []; perturb = 0;
fp = [0,0,1]; fu = [0,1,0]; fc = [0,0,0];
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'cx'; cx = v; case 'cy'; cy = v; case 'cz'; cz = v;
      case 'clip'; clip = v; case 'nonnegative'; nonnegative = v;
      case 'clean'; clean = v;
      case 'frame'; frame = v;
      case 'errormodel'; errormodel = v; case 'errorparams'; errorparams = v;
      case 'perturb'; perturb = v;
      case 'fp'; fp = v; case 'fu'; fu = v; case 'fc'; fc = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else warning('non-string %s, expected name',num2str(n));
  end
end

e = 0;
if (isscalar(cx)); cx = cx*ones(sz); elseif (any(size(cx)~=sz)); e = 1; end
if (isscalar(cy)); cy = cy*ones(sz); elseif (any(size(cy)~=sz)); e = 1; end
if (isscalar(cz)); cz = cz*ones(sz); elseif (any(size(cz)~=sz)); e = 1; end
if (e); error('supplied data must all be same size'); end

% transform input data to local frame if necessary
if (strcmpi(frame,'world'))
  wmx = mx; wmy = my; wmz = mz; wcx = cx; wcy = cy; wcz = cz;
  [mx, my, mz] = xform3(mx, my, mz, p.r, p.c, 1, 1); % invert, rotonly
  [cx, cy, cz] = xform3(cx, cy, cz, p.r, p.c, 1); % invert
end

% calculate ray intersections
r = p.ri(mx,my,mz,cx,cy,cz);

% clean them up
r(isnan(r)|(imag(r)~=0)) = inf;
if (nonnegative); r(r<0) = inf; end

if (clip); r(p.bl(cx+mx.*r,cy+my.*r)>0) = inf; end;

if (clean||om||ocov||perturb)
  if (strcmpi(frame,'world')) % go back to world frame
    mx = wmx; my = wmy; mz = wmz; cx = wcx; cy = wcy; cz = wcz;
  end
end

if (clean||ocov||perturb)
  vi = ~isinf(r);
  r = r(vi);
  mx = mx(vi); my = my(vi); mz = mz(vi);
  cx = cx(vi); cy = cy(vi); cz = cz(vi);
end

if (ocov||perturb)

  % transform to camera frame
  cfb = aim(fp(:),fu(:));
  [cmx,cmy,cmz,~] = samplecvt('mx',mx,'my',my,'mz',mz,'r',r,...
                              'xform',[cfb',[0 0 0]']);
  [cx,cy,cz] = xform3(cx,cy,cz,cfb,fc(:),1); %invert
  
  % generate covariance matrices in cam frame
  [cxx,cyy,czz,cxy,cyz,cxz] = rangecovar(cmx,cmy,cmz,r,...
                                         errormodel,errorparams);
  
  % xform covariance matrices back to world frame and add noise
  [mxx,myy,mzz,rr,cxx,cyy,czz,cxy,cyz,cxz] = ...
      samplecvt('mx',cmx,'my',cmy,'mz',cmz,'r',r,...
                'cx',cx,'cy',cy,'cz',cz,...
                'cxx',cxx,'cyy',cyy,'czz',czz,...
                'cxy',cxy,'cyz',cyz,'cxz',cxz,...
                'xform',[cfb,[0 0 0]'],...
                'perturb',perturb);

  if (perturb); mx = mxx; my = myy; mz = mzz; r = rr; end
  
end

varargout{1} = r;
if (om); varargout{2} = mx; varargout{3} = my; varargout{4} = mz; end
if (ocov)
  varargout{5} = cxx; varargout{6} = cyy; varargout{7} = czz;
  varargout{8} = cxy; varargout{9} = cyz; varargout{10} = cxz;
end

if (dbg); fprintf('patchsample: %gs\n',toc(tstart)); end
  
end
