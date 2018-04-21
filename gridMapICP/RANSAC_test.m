clc
clear

% Parameters
spNum = 20;
iter = 1000;
thDist = 3;

% Generate a random plane (z=ax+by+c)
noise = rand(100,1)*8-4;
plane = rand(3,1)*10-5;
pc = [rand(100,2)*100-50,ones(100,1)];
pc = [pc(:,1:2),pc*plane+noise]';

% % Generate a random plane (ax+by+cz=1)
% noise = rand(100,3)*4-2;
% plane = rand(3,1)*10-5;
% pc = [rand(100,2)*100-50];
% pc = [pc,(1-(pc*plane(1:2,1)))/plane(3)]';
% pc = pc+noise';

% Plane fitting by RANSAC
[best_plane,best_inl] = RANSAC(pc,spNum,iter,thDist);

figure
hold on
scatter3(pc(1,:),pc(2,:),pc(3,:),5,'k','filled')
[x,y] = meshgrid(-50:50:50,-50:50:50);
surf(x,y,x*best_plane(1)+y*best_plane(2)+best_plane(3),'FaceColor','r','FaceAlpha',0.5);
% surf(x,y,(1-x*best_plane(1)-y*best_plane(2))/best_plane(3),'FaceColor','r','FaceAlpha',0.5);
view(3)

disp(['Plane:  ',num2str(plane')])
disp(['RANSAC: ',num2str(best_plane')])
disp(['Noise mean: ',num2str(mean(noise))])
(plane./best_plane)'
