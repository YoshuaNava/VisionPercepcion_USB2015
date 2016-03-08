function [ss, hss, ssm, pp, hpp] = demomanseg()
% demomanseg() runs demospl() in manual segmentation mode
%
% Copyright (C) 2013 Marsette A. Vona

%fn = datafn('rock.ply'); sco = 3; % not organized
fn = datafn('newrock.pcd'); sco = 7; % grid organized, enables mesh and bfs

spo = 9; 
%spo = 10; % plot mesh

%spmax = 0;
spmax = 3000;

opts = {'sc',1,'scopts',sco,'fit',0,'pfopts',4,...
        'sp',0,'av',0,'dt',0,'spmax',spmax,'spopts',spo,...
        'smopts',3,'ms',1};

fprintf('manual segmentation: fitting paraboloids with ellipse boundary\n');

[ss, hss, ssm, pp, hpp] = demospl(fn,opts{:});

end
