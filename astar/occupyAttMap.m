function map_out = occupyAttMap( map, att )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
    s_map = size(map);
    n = size(att,1);
    cnt = 0;
    map_out = map;
    
    for i = 1:s_map(1)
       for j = 1:s_map(2)
           p = [i,j];
 %          if(isInside(obs, n,p))
            if(inpolygon(i,j,att(:,1),att(:,2)))
              cnt = cnt + 1;
              map_out(i,j) = 1;
           end
       end
    end
    
end

