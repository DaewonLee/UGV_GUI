function path_s = simplyfyPath( path, threshold )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
path_s = [];
Pstart = path(1,:);
Pnext = path(2,:);
Ptest = [path(3,:),3];
path_s=[path_s;Pstart];

for i=1:size(path,1)-2
   
    coef = findLineCoef(Pstart, Pnext);
    dist = findDistP2L(Ptest(1:2),coef);
    if (i==119)
        aa = 1;
    end
    if (dist < threshold)
        if (i<(size(path,1)-3))
            Ptest = [path(Ptest(3)+1,:),Ptest(3)+1];
        end
    else
        path_s=[path_s;path(Ptest(3)-1,:)];
        Pstart = path(Ptest(3)-1,:);
        Pnext = path(Ptest(3),:);
        if (i<(size(path,1)-3))
            Ptest = [path(Ptest(3)+1,:),Ptest(3)+1];
        end
    end   
end

path_s=[path_s;path(size(path,1),:)];

end

