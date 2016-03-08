function [ss, hss, ssm, pp, hpp] = demoautoseg(asopts,varargin)
% demoautoseg() runs demospl() in auto segmentation mode with newrock.pcd
%
%   Optional arg asopts selects autoseg options, default 5, see demospl.m
%
%   Additional optional args are passed on to demospl, e.g. use 'dp',0 to
%   disable drawing patches for timing.
%
% Copyright (C) 2013 Dimitrios Kanoulas and Marsette A. Vona

if (nargin<1); asopts = 5; end

fn = datafn('newrock.pcd');

spo = 9; 
%spo = 10; % plot mesh

%spmax = 0;
spmax = 3000;

opts = {'sc',1,'scopts',7,'fit',0,'pfopts',4,...
        'sp',0,'av',0,'dt',0,'spmax',spmax,'spopts',spo,...
        'smopts',3,'as',1,'asopts',asopts,varargin{:}};

fprintf('automatic segmentation: fitting paraboloids with ellipse boundary\n');

[ss, hss, ssm, pp, hpp] = demospl(fn,opts{:});

end
