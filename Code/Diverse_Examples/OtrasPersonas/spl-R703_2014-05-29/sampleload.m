function varargout = sampleload(fn,varargin)
% sampleload(fn) loads 3D data samples from file fn
%
%   Input argument fn must name a file of one of the currently supported
%   types.  See details in FORMATS, below.
%
%   Three, five, or six outputs may be requested:
%
%   x y z [u v [faces]]
%
%   The outputs x, y, and z are matrices of the same size MxN containing the
%   data point 3D coordinates.  The dimensions depend on the input format; for
%   example, when loading a range map the dimensions may match the original
%   sensor resolution.  But when loading general point cloud data the outputs
%   may each simply be 1xN.
%
%   The outputs u and v, if requested, are also of size MxN and give the texture
%   coordinates of each point, measured in (fractional) pixels.  Some formats
%   inherently encode these; otherwise they are synthesized as NaN.
%
%   The output faces, if requested, is an Fx1 cell array where F is the number
%   of faces in the file.  Each entry is a VARx1 column vector of indices into
%   the vectorized vertex data x(:), y(:), z(:), u(:), v(:).  If faces are
%   requested but no faces are available from the file then F=0 on return.
%
%   NOTE: The datatypes (single, double, etc) of the output arrays are chosen to
%   best match the types in the input file.
%
%   FORMATS
%
%   'ply' (polygon format): ply format is very general; this impl reads only
%   the common 'vertex' and 'face' elements. Other elements will be ignored.
%   Supported vertex properties are 'x', 'y', 'z', 'u', 'v', or any subset or
%   ordering thereof (others ignored).  Missing properties are read as 0.  Only
%   ascii 1.0 format is supported. '{' '}' delimited comments are allowed
%   anywhere.  Header lines starting with 'comment' are ignored.
%
%   'pcd' (point cloud data): pcd file format is used in the Point Cloud Library
%   (PCL, pointclouds.org).  The pcd format is described in the PCL
%   Documentation. In brief consists of a header that describes data properties
%   followed by the data (always row-major).  As of version 0.7 the header has
%   the following fields:
%
%     # comment
%     VERSION pcd file version # comment
%     FIELDS fields for each point; only 'x', 'y', 'z' are supported TBD
%     SIZE the size of each field in bytes
%     TYPE the type of each field; 'I' (int), 'U' (unsigned int), or 'F' (float)
%     COUNT number of elements each field has
%     WIDTH the width of point cloud dataset
%     HEIGHT the height of the point cloud dataset
%     VIEWPOINT the viewpoint tx ty tz qw qx qy qz
%     POINTS total number of points in the point cloud dataset
%     DATA the data format; only 'binary' is supported
%
%   e.g.
%
%     # .PCD v.7 - Point Cloud Data file format
%     VERSION .7
%     FIELDS x y z rgb
%     SIZE 4 4 4 4
%     TYPE F F F F
%     COUNT 1 1 1 1
%     WIDTH 213
%     HEIGHT 1
%     VIEWPOINT 0 0 0 1 0 0 0
%     POINTS 213
%     DATA binary
%     [213*(4+4+4+4) floats row-major]
%
%   'png' (single channel range image): a single channel depth image.  The value
%   of pixel is the depth from the camera in units of 1/5000 meter.  Camera
%   intrinsics are assumed to be those of the Kinect. For details see
%   http://vision.in.tum.de/data/datasets/rgbd-dataset
%
%   OPTIONS
%
%   A list of name,value pairs may be given to set options. Unrecognized
%   names cause warnings.  If a name is given more than once the last-given
%   (rightmost) value takes precedence.
%
%   Option 'fmt' (default []): Must either be [] or one of the file format
%   identifiers listed in the FORMATS section above to override the internal
%   file format detection mechanism.
%
%   Option 'nr' (default 0): if greater than zero the output matrices are
%   reformatted to (nr)x(nv/nr) column-major (first nr points go down first
%   column of returned matrix), where nv is the total number of vertices (nv
%   must be divisible by nr).  Mutually exclusive with 'nc'.
%
%   Option 'nc' (default 0): if greater than zero the output matrices are
%   reformatted to (nv/nc)x(nc) row-major (first nc points go across the first
%   row of returned matrix), where nv is the total number of vertices (nv must
%   be evenly divisible by nc).  Mutually exclusive with 'nr'.
%
%   Option 'dbg' (default 0): may have any nonnegative value. dbg=0 disables
%   debug. Larger values enable successively more verbose debug.
%
% Copyright (C) 2013 Dimitrios Kanoulas and Marsette A. Vona

tstart = tic();

% process name,value options
nva = length(varargin);
if (mod(nva,2)~=0)
  warning('expected even number of varargs (name,value pairs)');
  nva = nva-1;
end
nopt = nva/2;

% option defaults
fmt = []; nr = 0; nc = 0;
dbg = 0;

for i=1:nopt
  n = varargin{2*i-1}; v = varargin{2*i};
  if (ischar(n))
    switch (n)
      case 'fmt'; fmt = v; case 'nr'; nr = v; case 'nc'; nc = v;
      case 'dbg'; dbg = v;
      otherwise; warning('unexpected optional arg %s', n);
    end
  else warning('non-string, expected name'); end
end

if (isempty(fmt)); fmt = detectfmt(fn); end
if (isempty(fmt)); error('format detection failed'); end

switch (fmt)
  case 'ply'; vao = loadply(fn,nargout,dbg);
  case 'pcd'; vao = loadpcd(fn,nargout,dbg);
  case 'png'; vao = loadpng(fn,nargout,dbg);
  otherwise; error('fmt %s unsupported', fmt);
end

% synthesize u, v if requested but not read from file?
if ((nargout>=5)&&(length(vao)<5))
  sz = size(vao{1});
  vao{4} = nan(sz); vao{5} = nan(sz);
end

% faces requested but no faces loaded?
if ((nargout==6)&&(length(vao)<6)); vao{6} = []; end

% force output dimensions?
if (nr>0)
  if (nc>0); error('nr and nc are mutually exclusive'); end
  if ((nr~=round(nr))||mod(prod(size(vao{1})),nr))
    error('nv must be evenly divisible by nr');
  end
  for i=1:min(length(vao),5); vao{i} = reshape(vao{i},nr,[]); end
elseif (nc>0)
  if ((nc~=round(nc))||mod(prod(size(vao{1})),nc))
    error('nv must be evenly divisible by nc');
  end
  for i=1:min(length(vao),5); vao{i} = reshape(vao{i},nc,[])'; end
end

varargout = vao;

if (dbg); fprintf('sampleload: %gs\n',toc(tstart)); end

end % sampleload


function fmt = detectfmt(fn)
% detect file format; for now just returns file extension as lower case
fmt = lower(fn(find(fn=='.',1,'last')+1:length(fn)));
end


function o = loadply(fn,no,dbg)
% loads ply format, fn is filename, no is number of requested outputs

  function msg(varargin)
  if (dbg); fprintf(varargin{:}); end
  end

[fid, em] = fopen(fn,'rt');
if(fid == -1); error(em); end

  function lt = toknl()
  % extract a line of tokens in fmt and convert to lower case
  lt = {};
  while(isempty(lt));
    lt = textscan(fgetl(fid),'%s','commentstyle',{'{','}'});
    lt = lower(lt{1});
  end
  end

lt = toknl();
if(~strcmpi(lt{1},'ply')); fclose (fid); error('malformed ply'); end

lt = toknl();
if(~strcmpi(lt{1},'format')); fclose(fid); error('malformed ply'); end
if(~strcmpi(lt{2},'ascii')); fclose(fid); error('ply %s unsupported',lt{2}); end
if(~strcmpi(lt{3},'1.0')); fclose(fid); error('ply %s unsupported',lt{3}); end

% 1-base indices of each vertex property within a vertex, neg if none
xi = -1; yi = -1; zi = -1; ui = -1; vi = -1;

vs = -1; fs = -1; % 1-base starts of vertex and face datablocks, neg if none
nv = 0; nf = 0; vc = 0; % number of vertices, faces, vertex components
ni = 1; % number of input lines after header
en = []; % current element name

while (1) % parse header
  
  lt = toknl();

  switch (lt{1})
    case 'comment'; % ignore
    case 'element';
      switch (lt{2})
        case 'vertex';
          en = lt{2}; nv = str2double(lt{3});
          if (nv>0); vs = ni; ni = ni+nv; end
        case 'face';
          en = lt{2}; nf = str2double(lt{3});
          if (nf>0); fs = ni; ni = ni+nf; end
        otherwise;
          msg('W: ignoring ply element %s\n',lt{2});
          en = lt{2}; ni = ni+str2double(lt{3}); % skip
      end
    case 'property';
      if (strcmpi('vertex',en)) % we only care about vertex props
        switch (lt{3})
          case 'x'; xi = vc+1; case 'y'; yi = vc+1; case 'z'; zi = vc+1;
          case 'u'; ui = vc+1; case 'v'; vi = vc+1;
          otherwise; msg('W: ignoring ply vertex property %s\n',lt{3});
        end
        vc = vc+1;
      else msg('W: ignoring ply %s property %s\n',en,lt{3}); end
    case 'end_header'; break;
    otherwise; msg('W: ignoring ply header %s\n',lt{1}); % ignore
  end
end

ve = vs+nv-1; fe = fs+nf-1; % end indices, neg if none

ouv = (no>=5); of = (no>=6); % read uv and faces?

if (ouv&&((ui<0)||(vi<0)))
  msg('W: texcoords requested but not available in file\n');
  ouv = 0;
end

if (of&&(fs<0))
  msg('W: faces requested but not available in file\n');
  of = 0;
end

if (ouv); u = zeros(nv); v = zeros(nv); end
if (of); faces = cell(nf,1); end

i = 1;
while (i<=(ni-1)) % parse data
  if (i==vs) % parse vertices
    vd = textscan(fid,'%f',nv*vc,'commentstyle',{'{','}'}); vd = vd{1};
    fgetl(fid); % eat final newline
    vd = reshape(vd,vc,nv);
    i = i+nv; % read nv lines
    if (xi>0); x = vd(xi,:)'; else x = zeros(nv); end
    if (yi>0); y = vd(yi,:)'; else y = zeros(nv); end
    if (zi>0); z = vd(zi,:)'; else z = zeros(nv); end
    if (ouv) u = vd(ui,:)'; v = vd(vi,:)';
    end
  elseif (of&&(i>=fs)&&(i<=fe)) % parse face (num verts per face may vary)
    lt = textscan(fgetl(fid),'%f','commentstyle',{'{','}'});
    faces{i-fs+1} = lt; i = i+1;
  else % skip line
    fgetl(fid); i = i+1;
  end
end

fclose(fid);

o{1} = x; o{2} = y; o{3} = z;
if (ouv); o{4} = u; o{5} = v; end
if (of); o{6} = faces; end

end % loadply()


function o = loadpcd(fn,no,dbg)
% loads pcd format, fn is filename, no is number of requested outputs

  function msg(varargin)
  if (dbg); fprintf(varargin{:}); end
  end

[fid, em] = fopen(fn,'rt');
if(fid == -1); error(em); end

% each header line is ASCII and the lines are in a fixed order

  function lt = toknl()
  % extract a line of tokens in fmt and convert to lower case
  lt = {};
  while(isempty(lt));
    lt = textscan(fgetl(fid),'%s','commentstyle',{'#'});
    lt = lower(lt{1});
  end
  end

  function lt = hdr(name,lt)
  % extract a header line as lowercase string tokens and verify name
  if (nargin<2); lt = toknl(); end
  if (~strcmpi(lt{1},name))
    if (nargin<2)
      fclose(fid);
      error('malformed pcd header, got %s, expected %s', lt{1}, name);
    else lt = {}; end
  end
  end

ver=0.7; % if 'version' header missing assume 0.7
lt = toknl();
if (~isempty(hdr('version',lt)))
  ver = str2double(lt{2});
  lt = toknl();
elseif (~strcmpi(lt{1},'fields'));
  fclose(fid);
  error('malformed pcd header, got %s, expected version or fields', lt{1});
end

ver = round(10*ver);

if (~any([5,7]==ver)); msg('W: pcd version .%s not supported',ver); end

lt = hdr('fields',lt); nf = length(lt)-1; dname = lt(2:nf+1);
lt = hdr('size'); dsize = str2double(lt(2:nf+1));
lt = hdr('type'); dtype = lt(2:nf+1);
lt = hdr('count'); dcount = str2double(lt(2:nf+1));
lt = hdr('width'); nc = str2double(lt(2));
lt = hdr('height'); nr = str2double(lt(2));
if (ver>=7); lt = hdr('viewpoint'); vp = str2double(lt(2:length(lt))); end
lt = hdr('points'); nv = str2double(lt(2));
lt = hdr('data'); dfmt = lt(2);

if (nv ~= nc*nr)
  msg('W: pcd header inconsistent: width=%d, height=%d, points=%d',nc,nr,nv);
  nv = nc*nr; % trust the width and height fields
end

if (~strcmpi(dfmt,'ascii')&&~strcmpi(dfmt,'binary'))
  fclose (fid); error('unsupported pcd data format: %s', dfmt);
end

% 1-base indices of each vertex property within a vertex, neg if none
xi = -1; yi = -1; zi = -1; ui = -1; vi = -1;
vc = 0; % total number of scalars per vertex
for i = 1:nf
  switch (dname{i})
    case 'x'; xi = vc+1; case 'y'; yi = vc+1; case 'z'; zi = vc+1;
    case 'u'; ui = vc+1; case 'v'; vi = vc+1;
    case {'-','_'}; dname{i} = ['f' num2str(i)];
    otherwise; msg('W: pcd field %s not supported, ignoring\n', dname{i});
  end
  dc = dcount(i);
  switch (dname{i})
    case {'x','y','z','u','v'}; 
      if (dc>1); msg('W: pcd x,y,z,u,v field count %d not supported',dc); end
  end
  vc = vc+dc;
end

ouv = (no>=5); of = (no>=6); % read uv and faces?

if (ouv&&((ui<0)||(vi<0)))
  msg('W: texcoords requested but not available in file\n');
  ouv = 0;
end

if (of)
  msg('W: faces requested but not available in file\n');
  of = 0;
end

if (strcmpi(dfmt,'ascii')) % read ascii data

  vd = textscan(fid,'%f',nv*vc,'commentstyle',{'#'}); vd = vd{1};
  vd = reshape(vd,vc,nv);

  if (xi>0); x = vd(xi,:); else x = zeros(nv); end
  if (yi>0); y = vd(yi,:); else y = zeros(nv); end
  if (zi>0); z = vd(zi,:); else z = zeros(nv); end
  if (ouv); u = vd(ui,:); v = vd(vi,:); end

else % read binary data
  
  mmt = cell(nf,1); % mmap type corresponding to dtype, dsize
  for i=1:nf
    switch (dtype{i})
      case 'i';
        switch dsize(i)
          case 1; mmt{i} = 'int8';
          case 2; mmt{i} = 'int16';
          case 4; mmt{i} = 'int32';
          case 8; mmt{i} = 'int64';
          otherwise; error('pcd type i size %d unsupported', dsize(i));
        end
      case 'u';
        switch dsize(i)
          case 1; mmt{i} = 'uint8';
          case 2; mmt{i} = 'uint16';
          case 4; mmt{i} = 'uint32';
          case 8; mmt{i} = 'uint64';
          otherwise; error('pcd type u size %d unsupported', dsize(i));
        end
      case 'f';
        switch dsize(i)
          case 4; mmt{i} = 'single';
          case 8; mmt{i} = 'double';
          otherwise; error('pcd type f size %d unsupported', dsize(i));
        end
      otherwise; error('pcd type %s unsupported', dtype{i});
    end
  end
  
  s = dir(fn); ofs = s.bytes-(nv*sum(dsize.*dcount));
  fmt = cat(2,mmt,mat2cell([dcount,ones(nf,1)],ones(nf,1),2),dname);
  m = memmapfile(fn, 'Offset', ofs, 'Format', fmt, 'Repeat', nv);
  md = m.Data;

  if (isfield(md,'x')); x = [md.x]; x = x(1,:); else x = zeros(nv); end
  if (isfield(md,'y')); y = [md.y]; y = y(1,:); else y = zeros(nv); end
  if (isfield(md,'z')); z = [md.z]; z = z(1,:); else z = zeros(nv); end

  if (ouv); u = [md.u]; u = u(1,:); v = [md.v]; v = v(1,:); end

end % read binary data

% PCD data is stored row-major
o{1} = reshape(x,nc,nr)'; o{2} = reshape(y,nc,nr)'; o{3} = reshape(z,nc,nr)';
if (ouv); o{4} = reshape(u,nc,nr)'; o{5} = reshape(v,nc,nr)'; end

end % loadpcd()

function o = loadpng(fn,no,dbg)
% loads png format, fn is filename, no is number of requested outputs

% For 16-bit PNG depth files in TUM RGB-D format
fx = 525.0; fy = 525.0;% focal length x and y
cx = 320.5; cy = 240.5;% optical center x and y
ds = 1.0/5000.0; % depth scaling

depth = double(imread(fn));
sz = size(depth); nr = sz(1); nc = sz(2);
x = nan(sz); y = nan(sz); z = nan(sz);

ii = (depth > 0); z(ii) = depth(ii)*ds;
u = repmat(1:nc,nr,1); v = repmat((1:nr)',1,nc);
x(ii) = ((u(ii)-cx).*z(ii))/fx; y(ii) = ((v(ii)-cy).*z(ii))/fy;

ouv = (no>=5); of = (no>=6); % read uv and faces?

if (of); msg('W: faces requested but not available in file\n'); of = 0; end

o{1} = x; o{2} = y; o{3} = z;
if (ouv); o{4} = u; o{5} = v; end

end % loadpng()
