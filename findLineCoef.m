function coef = findLineCoef( p1, p2 )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
x1 = p1(1);
y1 = p1(2);
x2 = p2(1);
y2 = p2(2);

    if (x2 == x1)
        x1 = x1 + 0.000000001;
    end
    coef(1) = (y2-y1)/(x2-x1);
    coef(2) = -coef(1)*x1+y1;
end

