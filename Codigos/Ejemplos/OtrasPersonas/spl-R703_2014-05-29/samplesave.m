function samplesave(x,y,z,fn,varargin)
% samplesave(x,y,z,fn) saves 3D data samples to file fn
%
%   The inputs x, y, and z are matrices of the same size MxN containing the
%   data point 3D coordinates.
%
%   The data is saved in ply format.  TBD add more formats.
%
%   OPTIONS
%
%   A list of name,value pairs may be given to set options.  Unrecognized names
%   cause warnings.  If a name is given more than once the last-given
%   (rightmost) value takes precedence.
%
%   Option 'sentinel' (default nan): sentinel value to write if keepall for all
%   coords of a vertex that had any coord nan, inf, or complex on input.
%
%   Option 'keepall' (default 1): whether to pass invalid data in the
%   output (always as points with one or more nan coordinates), or to cull
%   invalid points.
%
%   Option 'dbg' (default 0): may have any nonnegative value.  dbg=0 disables
%   debug.  Larger values enable successively more verbose debug.
%
% Copyright (C) 2013 Dimitrios Kanoulas and Marsette A. Vona

tstart = tic();

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expect even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
sentinel = nan; keepall = 1;
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'sentinel'; sentinel = v; case 'keepall'; keepall = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s',n);
    end
  else
    warning('non-string, expected name');
  end
end

x = x(:); y = y(:); z = z(:);
nv = length(x);

[fid, msg] = fopen(fn,'wt');
if(fid<0); error(msg); end

fprintf(fid,'%s\n','ply');
fprintf(fid,'%s\n','format ascii 1.0');
fprintf(fid,'%s %d\n','element vertex',nv);
fprintf(fid,'%s\n','property float x');
fprintf(fid,'%s\n','property float y');
fprintf(fid,'%s\n','property float z');
fprintf(fid,'%s\n','end_header');

% TBD it is probably possible to do this without a for loop
for i=1:nv;
  
  xx = x(i); yy = y(i); zz = z(i);
  xyz = [xx yy zz];
  
  if (any(isnan(xyz))||any(isinf(xyz))||(any(imag(xyz)~=0)))
    xx = sentinel; yy = sentinel; zz = sentinel;
    xyz = [xx yy zz];
  end
  
  if (keepall||(~any(xyz==sentinel)))
    fprintf(fid,'%f %f %f\n',x(i),y(i),z(i));
  end
end

if (dbg) fprintf('samplesave: %gs\n',toc(tstart)); end

end
