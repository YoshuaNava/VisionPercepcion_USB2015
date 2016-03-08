function [p t u v] = setcam(varargin)
% [p t u v] = setcam(a,p,t,u,v) sets camera settings for axes a
%
%  May be called with 1, 2, 4, or 5 arguments.
%
%  In the 1 or 2 argument case the last argument is interpreted as a filename if
%  it's a string, in which case the file must be in the same format as saved by
%  getcam().  Otherwise it must be a 4x3 matrix [p;t;u;v] where p is the camera
%  position, t is the camera target, u is the camera up vector, and v(1) is the
%  camera view angle.  If 2 arguments are given the first is the axes to
%  configure, otherwise the current axes are used.
%
%  In the 4 or 5 argument case the last 4 arguments are p, t, u, v.  The last
%  may be either scalar or 3x1; the others must be 3x1 or 1x3.  If 5 arguments
%  are given the first is the axes to configure, otherwise the current axes are
%  used.
%
%  Returns previous settings either as 4 1x3 vectors or one 4x3 matrix
%  [p;t;u;v].
%
% Copyright (C) 2013 Marsette A. Vona

va = varargin; na = nargin; a = []; fn = []; s = [];
switch (na)
  case 1; a = gca(); fn = va{1}; s = va{1};
  case 2; a = va{1}; fn = va{2}; s = va{2};
  case 4; a = gca();
  case 5; a = va{1};
  otherwise; error('expected 1, 2, 4, or 5 inputs, got %d',na);
end

if (ischar(fn)); s = load(fn,'-ascii'); end

if (~isempty(s))
  p = s(1,:); t = s(2,:); u = s(3,:); v = s(4,1);
else % take last 4 args
  p = va{na-3}; p = p(:)';
  t = va{na-2}; t = t(:)';
  u = va{na-1}; u = u(:)';
  v = va{na}; v = v(1);
end

swas = getcam(a);

set(a,'CameraPosition',p);
set(a,'CameraTarget',t);
set(a,'CameraUpVector',u);
set(a,'CameraViewAngle',v);

p = swas(1,:); t = swas(2,:); u = swas(3,:); v = swas(4,:);

if (nargout<=1); p = swas; end

end
