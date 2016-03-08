function [cr,ct,cp,pr,gg,rr,tt,ii] = samplepick(keyhook,dsel,pick)
% [cr,ct,cp,pr,gg,rr,tt,ii] = samplepick() interactively pick from a sampleplot
%
%   Implements click-and-drag selection for orthographic cameras in the current
%   axes.  Existing mouse handlers are temporarily replaced and the user may
%   repeatedly pick data until hitting q, return, or escape.  A key release hook
%   may be given to catch other keypresses, e.g. to process the current pick.
%
%   Existing event handlers that were replaced on call will be restored
%   on return.
%
%   Optional arg keyhook is either null (the default) or a function handle with
%   the signature
%
%     hook(key,cr,ct,cp,pr,gg,rr,tt,ii)
%
%   that will be called on unhandled key release events.  The final argument ii
%   is optional and may be omitted, see below.
%
%   Optional arg dsel indicates the sampleplot() dataset(s) in the axes that
%   should be considered for picking.  If omitted or null (the default) then all
%   sampleplot datasets are considered.  If 'first' then only the first dataset
%   found is used.  Otherwise a scalar or vector of handles to the scatter3
%   objects tagged 'samples' that are children of hggroups returned by
%   samplepolot.
%
%   Optional arg pick is either null (the default) or a function handle with
%   the signature
%
%   ii = pick(r,c,d,s)
%
%   where r is a scalar pick radius, c is a Dx1 pick center point, d is a DxN
%   data matrix, and s is the original data matrix size RxC s.t. N = R*C.  The
%   default pick function is cylinder selection using inrball().
%
%   Returns cr and ct give an xform from world to camera coordinates as of
%   the beginning of the (last) pick.
%
%   Return cp is the center 'front' point of the (last) pick in camera frame and
%   pr is the pick radius.  cp will be empty before the first pick begins.
%
%   Returns gg, rr, and tt are Sx1 cell arrays where S is the number of 
%   sampleplot datasets selected for picking. gg gives the tag of the
%   dataset and rr, tt give its local-to-camera transform at the start of the
%   last pick.
%
%   Return ii is an Sx1 cell array where each entry is an array of the picked
%   indices in the corresponding dataset.  This can be large, and may not always
%   be useful as the indices refer only to displayed data, which in some cases
%   may only be a subset of the full data.
%
% Copyright (C) 2013 Marsette A. Vona

hcolor = 'r'; htweak = 1e-3; % highlight config

if (nargin<3); pick = []; end
if (nargin<2); dsel = []; end
if (nargin<1); keyhook = []; end

  function ii = cylpick(r,c,d,s)
  ii = inrball(r,c(1:2),d(1:2,:));
  end

if (isempty(pick)); pick = @cylpick; end

a = gca(); f = get(a,'Parent'); % current axes and figure

if (~strcmpi(get(a,'Projection'),'orthographic'))
  error('samplepick requires orthographic projection');
end

udwas = get(a,'UserData'); set(a,'UserData','samplepickon');

% init world to camera frame xform
[cr,ct] = camxform(a,1);

% collect datasets
sph = findobj(a,'Tag','samples');
if (isempty(dsel)); ss = sph;
elseif (ischar(dsel)&&strcmpi('first',dsel)&&~isempty(sph)) ss = sph(1);
else ss = intersect(dsel,sph); end

nss = length(ss); % num datasets

% per-dataset state
sz = cell(nss,1); % original data matrix dimensions
pp = cell(nss,1); % 3xN data in local frame
cpp = cell(nss,1); % 3xN data in camera frame
hs = cell(nss,1); % handle to highlight scatter
ii = cell(nss,1); % indices of picked points
gg = cell(nss,1); % tag
rr = cell(nss,1); % local to camera rotation matrix
tt = cell(nss,1); % local to camera translation vector

% data in local frame
for i=1:nss

  x = get(ss(i),'XData'); y = get(ss(i),'YData'); z = get(ss(i),'ZData');

  ud = get(ss(i),'UserData');
  if (isvector(x)&&~isempty(ud)&&isequal(size(ud),[1,2])); sz{i} = ud;
  else sz{i} = size(x); end

  pp{i} = [x(:),y(:),z(:)]';

  gg{i} = get(ss(i),'Tag');
end

dragon = 0; % ha
cp = []; % camera frame point where button went down, null until first click
pr = 0; % pick radius

  function reproject() % reproject datasets to camera frame
  for i=1:nss
    rr{i} = eye(3,3); tt{i} = zeros(3,1);
    p = get(ss(i),'Parent');
    while (~isempty(p))
      if (strcmpi('hgtransform',get(p,'Type')))
        m = get(p,'Matrix'); r = m(1:3,1:3); t = m(1:3,4);
        rr{i} = r*rr{i}; tt{i} = r*tt{i}+t; % local to world
      end
      p = get(p,'Parent');
    end % traversing parents
    rr{i} = cr*rr{i}; tt{i} = cr*tt{i}+ct; % world to camera
    [~,n] = size(pp{i}); cpp{i} = rr{i}*pp{i}+repmat(tt{i},1,n); 
  end % for each dataset
  end % reproject()

  function repick() % re-pick and update highlight
  t=[0 0 -htweak]*cr;
  for i=1:nss
    if (ishandle(hs{i})); delete(hs{i}); hs{i} = []; end
    ii{i} = pick(pr, cp, cpp{i}, sz{i});
    if (~isempty(ii{i}))
      p = pp{i}(:,ii{i});
      s = ss(i);
      if (isscalar(ii{i})); mk = 'o'; else mk = get(s,'Marker'); end
      hs{i} = scatter3(p(1,:)+t(1),p(2,:)+t(2),p(3,:)+t(3),...
                       'Tag','samplepick',...
                       'Marker',mk,...
                       'MarkerFaceColor',hcolor,'MarkerEdgeColor',hcolor,...
                       'SizeData',1.5*1.5*get(s,'SizeData'));
    end % non-empty pick
  end % for each dataset
  end

  function p = fp() % get current mouse 'front' point in camera frame
  p = get(a, 'CurrentPoint'); p = cr*p(1,:)'+ct;
  end

  function buttondown(src,ed) % start drag
  crwas = cr; ctwas = ct; [cr,ct] = camxform(a,1);
  if (~isequal(crwas,cr)||~isequal(ctwas,ct)); reproject(); end
  cp = fp();
  % pr = 0; leave radius from last pick if any in case of a click with no drag
  dragon = 1;
  end

  function buttonup(src,ed)
  if (dragon); repick(); end
  dragon = 0;
  end

  function buttonmotion(src,ed) % update pick and highlight
  if (dragon); p = fp(); pr = norm(p(1:2)-cp(1:2)); repick(); end
  end

  function keyrelease(src,ed) % handle keypresses
  switch (ed.Key)
    case {'q','escape','return'};
      fprintf('stopped picking samples\n');
      for i=1:nss; if (ishandle(hs{i})) delete(hs{i}); end; end
      % remove picking callbacks
      set(f,'WindowButtonDownFcn',wbdwas);
      set(f,'WindowButtonUpFcn',wbuwas);
      set(f,'WindowButtonMotionFcn',wbmwas);
      set(f,'WindowKeyReleaseFcn',wkrwas);
      set(a,'UserData','samplepickoff');
    otherwise; % ignore
  end % key dispatch
  if (~isempty(keyhook))
    args = {ed.Key,cr,ct,cp,pr,gg,rr,tt};
    if (nargin(keyhook)>length(args)); args = {args{:},ii}; end
    keyhook(args{:});
  end % call keyhook
  end % keyrelease()

  function cbwas = setcb(n,cb); % set a callback function
  cbwas = get(f,n); set(f,n,cb);
  end

reproject(); % init datasets in camera frame

% setup picking callbacks
wbdwas = setcb('WindowButtonDownFcn',@buttondown);
wbuwas = setcb('WindowButtonUpFcn',@buttonup);
wbmwas = setcb('WindowButtonMotionFcn',@buttonmotion);
wkrwas = setcb('WindowKeyReleaseFcn',@keyrelease);

fprintf('drag to pick samples, q|escape|return=quit\n');

try
  waitfor(a,'UserData','samplepickoff');
  set(a,'UserData',udwas);
catch ME
  % e.g. user closed window
end

end


