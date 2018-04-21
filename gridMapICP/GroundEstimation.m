clc
clear
load('../read CPEV data/CPEV170913/out.mat')

% Parameters
spNum = 20;
iter = 50;
z_th = 0.2;


%% PC of the target frame
pc = raw(:,raw(1,:)==1);
[x,y,z] = sph2cart(pc(3,:),pc(5,:),pc(4,:));
P = [x;y;z];
P_ground_init = P(:,P(3,:)<=0);

% Sample
t = tic;
tt = 0;
xrange = (P(1,:)>-2.5 & P(1,:)<-1.5) | (P(1,:)>-0.5 & P(1,:)< 0.5) | (P(1,:)> 1.5 & P(1,:)< 2.5);
yrange = (P(2,:)>4.5 & P(2,:)<5.5) | (P(2,:)>9 & P(2,:)<11) | (P(2,:)>-9.5 & P(2,:)<-8.5) | (P(2,:)>-13 & P(2,:)<-11);
sample = P(:,xrange & yrange)';
A = [sample(:,1:2),ones(size(sample,1),1)];
disp(['Sample point number: ',num2str(size(sample,1))])
disp(['Sampling takes ',num2str(toc(t)-tt)])
tt = toc(t);

% Plane fitting by least square
plane = (A'*A)\A'*sample(:,3);
disp(['Least-square method takes ',num2str(toc(t)-tt)])
tt = toc(t);

% Find ground points
z_bar = ([P(1:2,:);ones(1,size(P,2))])'*plane;
P_ground = P(:,abs(z_bar'-P(3,:))<z_th);
disp(['Ground point estimation takes ',num2str(toc(t)-tt)])
tt = toc(t);
disp(['Total:  ',num2str(tt)])


% Show the scatter before transformed
figure
hold on
scatter3(P_ground_init(1,:),P_ground_init(2,:),P_ground_init(3,:),5,'k','filled')
scatter3(P_ground(1,:),P_ground(2,:),P_ground(3,:),5,'c','filled')
scatter3(sample(:,1),sample(:,2),sample(:,3),5,'r','filled')
% [x y] = meshgrid(-10:10:10,-20:10:20);
% z = plane(1)*x+plane(2)*y+plane(3);
% surf(x,y,z,'FaceColor','c','FaceAlpha',0.5);
axis equal
legend('Raw (lower part)','Estimated Ground','Sample')
title('Least-square method')

% Plane fitting by RANSAC
[plane,inliers] = RANSAC(sample',spNum,iter,z_th);
% Find ground points
z_bar = ([P(1:2,:);ones(1,size(P,2))])'*plane;
P_ground = P(:,abs(z_bar'-P(3,:))<z_th);
figure
hold on
scatter3(P_ground_init(1,:),P_ground_init(2,:),P_ground_init(3,:),5,'k','filled')
scatter3(P_ground(1,:),P_ground(2,:),P_ground(3,:),5,'c','filled')
scatter3(sample(:,1),sample(:,2),sample(:,3),5,'r','filled')
% [x y] = meshgrid(-10:10:10,-20:10:20);
% z = plane(1)*x+plane(2)*y+plane(3);
% surf(x,y,z,'FaceColor','c','FaceAlpha',0.5);
axis equal
legend('Raw (lower part)','Estimated Ground','Sample')
title('RANSAC')