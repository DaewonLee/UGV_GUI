clear all
close all

p1 = [0,0];
p2 = [0,1];

coef = findLineCoef(p1,p2);

p = [-3,7];
dist = findDistP2L(p,coef);

path = [1,1; 2,2; 3,3; 4,4; 3,5; 2,6; 1,7; 2,9];

thresh_dist = 1.0;
path_s = simplyfyPath( path, thresh_dist );

