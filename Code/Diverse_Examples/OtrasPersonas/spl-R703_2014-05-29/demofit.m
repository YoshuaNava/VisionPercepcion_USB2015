function [ss, hss, ssm, pp, hpp] = demofit(tpi)
% demofit() runs demospl() with rock.ply and rockpatch?.ply
%
%   tpi (default 1:6) gives indices of rockpatch?.ply to load
%
%   rock.ply and rockpatch?.ply are all plotted into the same axes and the
%   patches are each fit.
%
% Copyright (C) 2013 Marsette A. Vona

if (nargin<2); tpi = 1:6; end; np = length(tpi);

fn = {'rock.ply'}; % whole rock is first dataset

%spmax = 0;
spmax = 30000;

sco = ones(np+1,1); sco(1) = 3; % sample convert opts
fit = ones(np+1,1); fit(1) = 0; % fit patches but not whole rock

opts = {'sc',1,'scopts',sco,'fit',fit,'pfopts',4,...
        'sp',0,'av',0,'dt',0,'spmax',spmax};

if (np>0)
  pfn = cellfun(@(i)(sprintf('rockpatch%d.ply',i)),num2cell(tpi),...
                'UniformOutput',0);
  fn = [fn,pfn];
end

fn = cellfun(@datafn,fn,'UniformOutput',0);

[ss, hss, ssm, pp, hpp] = demospl(fn,opts{:});

end
