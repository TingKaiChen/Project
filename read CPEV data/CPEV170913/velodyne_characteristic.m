clc
clear

load('./out.mat')

pc = raw(:,raw(1,:)==1);
azi = pc(3,:);
ele = pc(5,:);
r = pc(4,:);
[x,y,z] = sph2cart(azi,ele,r);

figure
plot(rad2deg(azi),'-o')
title('Azimuth (Frame #1)')
xlabel('laser ID')
ylabel('degree')
saveas(gcf,'../../figure/velodyne_azi.png')

figure
plot(rad2deg(ele),'-o')
title('Elevation (Frame #1)')
xlabel('laser ID')
ylabel('degree')
xlim([10 90])
saveas(gcf,'../../figure/velodyne_ele.png')