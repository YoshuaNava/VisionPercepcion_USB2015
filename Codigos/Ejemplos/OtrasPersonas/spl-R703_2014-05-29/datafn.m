function fn = datafn(fn)
% fn = datafn(fn) locates a data file
%
%   If the input filename it is passed directly to the output if it looks like
%   an absolute path or it refers to an existing file when resolved against the
%   current working directory.  Otherwise prepends the absolute path to the
%   "data" subdirectory in the directory containing this function.
%
% Copyright (C) 2013 Marsette A. Vona

if (isempty(fn)||((fn(1)~='/')&&(fn(1)~='\')&&isempty(dir(fn))))
  %tb = getfield(what('spl'),'path'); % breaks if toolbox name changes
  tb = mfilename('fullpath');
  tb = tb(1:(find(tb=='/',1,'last')-1));
  fn = [tb,'/data/',fn]; % pathsep / ok even on windows
end

end
