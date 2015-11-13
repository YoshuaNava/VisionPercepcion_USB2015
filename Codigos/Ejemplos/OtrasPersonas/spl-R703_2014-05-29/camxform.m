function [r, t] = camxform(a, invert)
% camxform(a) calculates the camera-to-world transform for axes a
%
%   If no input argument is given then the current axes are used.
%
%   Set input argument invert to 1 to return the world-to-camera transform.
%   If omitted invert defauts to 0.
%
%   Output r is a 3x3 rotation matrix whose columns are the basis of camera
%   frame (x right, y down, z out) and t a 3x1 vector giving the location of
%   the camera center.
%
% Copyright (C) 2013 Marsette A. Vona

if (nargin < 2); invert = 0; end
if (nargin < 1); a = gca(); end

cp = get(a,'CameraPosition'); % row vector
ct = get(a,'CameraTarget'); % row vector
cu = get(a,'CameraUpVector'); % row vector

cz = ct-cp; cz = cz/norm(cz);
cx = cross(cz,cu); cx = cx/norm(cx);
cy = cross(cz, cx);

r = [cx;cy;cz]';
t = cp';

if (invert); r = r'; t = -r*t; end

end

