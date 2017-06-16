function f = Rdes (x,v)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
sx = sin(x(1));
sy = sin(x(2));
cx = cos(x(1));
cy = cos(x(2));
r  = [sqrt(2)/2*(sy+cy*sx) sqrt(2)/2*(sy-cy*sx) cx*cy];
f = [(r(1) - v(1));
     (r(2) - v(2))];
f = f'*f;

end

