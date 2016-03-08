function subtitle(s)
% subtitle(s) sets current axes subtitle
%
%  subtitle([]) removes any subtitle
%
% Copyright (C) 2013 Marsette A. Vona

a = gca();
t = get(get(a,'Title'),'String');
if (iscell(t)); t = t{1}; end
title(a,{t;s});

end
