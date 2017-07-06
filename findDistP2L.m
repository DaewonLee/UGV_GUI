function dist = findDistP2L( p, coef )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

px = p(1);
py = p(2);

a = coef(1);
b = coef(2);

dist = abs(a*px-py+b)/sqrt(a^2+1);


end

