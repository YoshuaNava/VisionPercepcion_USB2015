function [p t u v] = getcam(a,fn)
% [p t u v] = getcam(a) gets camera settings for axes a
%
%  Returns either 4 1x3 vectors or one 4x3 matrix [p;t;u;v].  p is the camera
%  position, t is the camera target, u is the camera up vector, and v(1) is the
%  view angle.
%
%  If a is omitted then the current axes are used.
%
%  fn is an optional filename to which the settings are saved as a 4x3 ascii
%  matrix.
%
% Copyright (C) 2013 Marsette A. Vona

if (nargin<1); a = gca(); end

p = get(a,'CameraPosition');
t = get(a,'CameraTarget');
u = get(a,'CameraUpVector');
v = get(a,'CameraViewAngle'); v = [v,0,0];

s = [p;t;u;v];

if (nargin>1); save(fn,'s','-ascii'); end

if (nargout<=1); p = s; end

end
