function [] = plotEarthCourse(lats,lons)
%% Show course on earth map

figure;
earthmap;
axis equal; hold on; view([-114 26])
% earthmap x,y,z -> (lat,lon) mapping are x->(0,-180),y->(0,-90),z->(90,0)
Xtraj = cos(lats*pi/180.0).*cos(lons*pi/180.0-pi);
Ytraj = cos(lats*pi/180.0).*sin(lons*pi/180.0-pi);
Ztraj=sin(lats*pi/180.0);
plot3(Xtraj,Ytraj,Ztraj,'r','linewidth',2);