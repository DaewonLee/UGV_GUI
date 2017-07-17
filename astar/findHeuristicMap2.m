function map_out = findHeuristicMap2(costmap_att, sMap,goal )
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

    for i=1:sMap(1)
        for j=1:sMap(2)
            
            map_out(i,j) = findDistance([i,j],goal);
            if ( costmap_att(i,j) == 1)
                map_out(i,j) = 0.01*map_out(i,j);
            end
        end
    end


end

