clc
clear
load('../read CPEV data/CPEV170913/out.mat')

%% PC of the target frame
pc = raw(:,raw(1,:)==1);
ground_0 = pc_filter(pc,0);
ground_2 = pc_filter(pc,-2);
ground_24 = pc_filter(pc,-2.4);
[x,y,z] = sph2cart(pc(3,:),pc(5,:),pc(4,:));
[x_0,y_0,z_0] = sph2cart(ground_0(3,:),ground_0(5,:),ground_0(4,:));
[x_2,y_2,z_2] = sph2cart(ground_2(3,:),ground_2(5,:),ground_2(4,:));
[x_24,y_24,z_24] = sph2cart(ground_24(3,:),ground_24(5,:),ground_24(4,:));
P = [x;y;z];
P_0 = [x_0;y_0;z_0];
P_2 = [x_2;y_2;z_2];
P_24 = [x_24;y_24;z_24];

% Show the scatter before transformed
figure
hold on
scatter3(P(1,:),P(2,:),P(3,:),5,'k','filled')
scatter3(P_24(1,:),P_24(2,:),P_24(3,:),5,'m','filled')
scatter3(P_2(1,:),P_2(2,:),P_2(3,:),5,'b','filled')
scatter3(P_0(1,:),P_0(2,:),P_0(3,:),5,'r','filled')
axis equal
% zlim([-10 10])
legend('Raw','z\_th=-2.4','z\_th=-2','z\_th=0')
