function axescfg(a,ag,ar,av,kv,gr)
% axescfg(a,ag,ar) - configure an axes for 3d graphics
%
%   a is the axes to configure
%
%   ag is whether to turn on the axes grid, default 0
%
%   ar is whether to turn on interactive rotation, default 1
%
%   av is whether the axes should be visible, optional, default 0
%
%   kv is whether to simulate an original kinect viewpoint, optional, default 0
%
%   gr is the amount to grow the bounding box in every dir, default 1
%
% Copyright (C) 2013 Marsette A. Vona

if (nargin<6); gr = 1; end
if (nargin<5); kv = 0; end
if (nargin<4); av = 0; end
if (nargin<3); ar = 1; end
if (nargin<2); ag = 0; end

f = get(a,'Parent');

set(f,'Renderer','opengl')
set(f,'Color','w');

if (ag); grid(a,'on'); else grid(a,'off'); end

xlabel(a,'x'); ylabel(a,'y'); zlabel(a,'z');

% 1:1:1 aspect ratio
set(a,'DataAspectRatioMode','manual','DataAspectRatio',[1 1 1], ...
      'PlotBoxAspectRatioMode','manual','PlotBoxAspectRatio',[1 1 1]);

if (~kv)

  view(a,3); % set default 3d viewpoint
  
  set(a,'CameraViewAngle',get(a,'CameraViewAngle')); % no stretch-to-fit
  set(a,'Projection','orthographic'); % ortho is default, but make sure
  
  fitbbox(a,gr);

else % simulate kinect viewpoint

  set(a,'Projection','perspective');

  xywh = get(f, 'Position');
  set(f,'Position',[xywh(1) xywh(2) 640 480]);

  b = hgbbox(a);
  if (b(6)>b(5)); zc = (b(5)+b(6))/2; else zc = 1; end

  setcam(a, [0 0 0], [0 0 zc], [0 -1 0], 62);

end

% turn on navigation controls
%if (ar); rotate3d(f,'on'); else rotate3d(f,'off'); end % simpler
if (ar); cameratoolbar(f,'NoReset','Show'); % better
else cameratoolbar(f,'Hide'); end

if (av); set(a,'Visible','on'); else set(a,'Visible','off'); end

end
