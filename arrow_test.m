close all
clear all

x = [-75.1896 -75.1896];

y = [39.9513 39.9513];

drawArrow(x1,y1,[-75.1904, -75.1892],[39.9511, 39.9516],{'Color','b','LineWidth',3}); hold on


props = {'Color','b','LineWidth',3};

function [ h ] = drawArrow( x,y,xlimits,ylimits,props )

 xlim([-75.1904, -75.1892])
 ylim([39.9511, 39.9516])

h = annotation('arrow');
set(h,'parent', gca, ...
    'position', [x(1),y(1),x(2)-x(1),y(2)-y(1)]);

end

