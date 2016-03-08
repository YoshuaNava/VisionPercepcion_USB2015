function [x1, x2] = quadsol(a, b, c)
% quadsol(a, b, c) solves for the roots of the quadratic equation
%
%   The input arguments a, b, c, must be matrices of the same size.  The outputs
%   x1, x2 will also be matrices of that size.  x1 and x2 are the solutions to
%   the (vector) quadratic equations a*x^2+b*x+c = 0.  They may be complex, of
%   course.  In the special case that a is zero x1 will be infinite but x2 will
%   give the single correct solution.
%
%   The calculation follows suggestions in NRC.
%
% Copyright (C) 2013 Marsette A. Vona

q = -0.5*(b+sign(b).*sqrt(b.*b-4*a.*c));

x1 = q./a; x2 = c./q;

end
