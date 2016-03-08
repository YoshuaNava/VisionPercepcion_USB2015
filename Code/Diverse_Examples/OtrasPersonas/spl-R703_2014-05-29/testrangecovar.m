function testrangecovar(model,k,ppo)
% testrangecovar() tests rangecovar(model,k) on samples of a patch
%
%   The samples are generated with samplefrustum() with parameters similar
%   both to kinect and to BB2, but with only 10x10 measurement rays.
%
%   ppo is a cell array of name, value pairs to pass to patchplot.
%
%   testrangecovar(model, k, dp) uses default ppo
%
% Copyright (C) 2013 Marsette A. Vona

defppo = {'df',0,'aw',3,'as',1.5,'bw',2,'gw',1, 'gr',1.3};
%defppo = {'df',0,'da',0,'aw',3,'as',1.5,'bw',2,'gw',1,'ss',0.5};
cpo = {'p',0.95,'de',2,'da',0};
spo = {};
%spo = {'dr',1,'da',0};
pp = 0.95; % confidence interval
dca = 1; % covar axes

if (nargin < 1) model = []; end
if (nargin < 2) k = []; end
if (nargin < 3) ppo = defppo; end

% tilted planar patch
%p.s = 'p'; p.b = 'r';
%p.d = [8 5];
%p.c = [15 0 0]; p.r = -(pi/6)*[0 1 0];

% elliptic paraboloid
p.s = 'e'; p.b = 'e'; p.k = [-0.1 -0.2];
p.d = [4 2.5];
p.c = [8 0 0]; p.r = -(pi/3)*[0 1 0];

% sample frustum, similar fovs to kinect and BB2
cx = 0; cy = 0; cz = 0;
fc = [cx, cy, cz];
fh = 60*pi/180; fv = 50*pi/180; fp = [1 0 0]; fu = [0 0 1];
nh = 10; nv = 10;
%nh = 8; nv = 5;
[mx, my, mz] = samplefrustum(fh, fv, nh, nv, fp, fu);

% generate noisless samples and sample covariance matrices
[r,mx,my,mz,cxx,cyy,czz,cxy,cyz,cxz] = ...
    patchsample(p,mx,my,mz,'cx',cx,'cy',cy,'cz',cz,'clean',1,...
                'errormodel',model,'errorparams',k,'fp',fp,'fu',fu,'fc',fc);

% noiseless xyz samples
dx = cx+mx.*r; dy = cy+my.*r; dz = cz+mz.*r;

% add noise
[x,y,z] = ...
    samplecvt('x',dx,'y',dy,'z',dz,...
              'cxx',cxx,'cyy',cyy,'czz',czz,'cxy',cxy,'cyz',cyz,'cxz',cxz,...
              'perturb',1);

% setup figure window
persistent fig;
if (isempty(fig)); fig = figure(); end
figure(fig); clf();% make fig current
set(fig,'Color','w');

patchplot(p,ppo{:});
hold('on');
sampleplot('x',x,'y',y,'z',z,...
           'frustum',[fh, fv, fp, fu],'cx',cx,'cy',cy,'cz',cz,...
           spo{:});
covarplot('cxx',cxx,'cyy',cyy,'czz',czz,'cxy',cxy,'cyz',cyz,'cxz',cxz,...
          'cx',dx,'cy',dy,'cz',dz,cpo{:});

end

