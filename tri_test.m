clear all
close all
scale = 0.0005;
location = [-75.1564, 39.9494];
tri = scale * [0,1; -0.3, -0.3; 0,0; 0.3, -0.3];
psi = 30 * pi/180;
rot = [cos(psi), sin(psi); -sin(psi), cos(psi)];
tri = tri * rot;

tri(:,1) = tri(:,1)+location(1);
tri(:,2) = tri(:,2)+location(2);

h_=fill(tri(:,1),tri(:,2),'b',...
    'facealpha',0.5,...
    'LineWidth',1);
axis([-1 1 -1 1]);

location = [-0.5, 0.5];
tri = scale * [0,1; -0.3, -0.3; 0,0; 0.3, -0.3];
psi = 270 * pi/180;
rot = [cos(psi), sin(psi); -sin(psi), cos(psi)];
tri = tri * rot;

tri(:,1) = tri(:,1)+location(1);
tri(:,2) = tri(:,2)+location(2);

set(h_,'XData',tri(:,1));
set(h_,'YData',tri(:,2));
