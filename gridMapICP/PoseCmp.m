clc
clear

load('./pose_graph/LoopClosure/lc_20170523162617.mat')
v_s1    = vertex;
e_s1    = edges;
load('./pose_graph/LoopClosure/lc_20170523162617.mat')
v_s3    = vertex;
e_s3    = edges;
load('./g2o_optimizer/LoopClosure/result_lc_20170523162617.mat')
v_cycle = vertex;
e_cycle = edges;

% Origin
figure
hold on
PtSize = 5;
scatter3(v_cycle(:,2),v_cycle(:,3),v_cycle(:,4),PtSize,'r','filled')
axis equal
xlim([min([v_s1(:,2);v_s3(:,2);v_cycle(:,2)])-25, max([v_s1(:,2);v_s3(:,2);v_cycle(:,2)])+25])
ylim([min([v_s1(:,3);v_s3(:,3);v_cycle(:,3)])-25, max([v_s1(:,3);v_s3(:,3);v_cycle(:,3)])+25])
zlim([min([v_s1(:,4);v_s3(:,4);v_cycle(:,4)])-15, max([v_s1(:,4);v_s3(:,4);v_cycle(:,4)])+15])
xlabel('X')
ylabel('Y')
zlabel('Z')

% GPS trajectory
load('../read CPEV data/CPEV170523/CPEV_Record_2017_05_23_16_26_17_gps.mat');
[x,y,z]=deg2utm(gpsdata(:,2),gpsdata(:,1));
% stFrame = 1;
stFrame = 3;	%20170522133150, 20170523162221, 20170523162617 start from frame #3
% Shift to origin
x = x-x(stFrame,1)+0.5;
y = y-y(stFrame,1)-4.5;
rot=eul2rotm([-0.83 0 0]);
rot=rot(1:2,1:2);
gps=rot*[x y]';

scatter(gps(1,:),gps(2,:),5,'b','filled');
legend('Optimized','GPS')
ylim([-110 380])


% Calculate the difference
Distance_sq  = (v_cycle(:,2:4)-v_s1(:,2:4)).^2;
Distance     = sum(Distance_sq,2).^0.5;
Distance_Sum = sum(sum(Distance_sq,2).^0.5)
max(Distance)
mean(Distance)

se_dist_s1    = sum((v_s1(1,2:4)-v_s1(end,2:4)).^2).^0.5
se_dist_cycle = sum((v_cycle(1,2:4)-v_cycle(end,2:4)).^2).^0.5

disp(['s1: 	  start: ', num2str(v_s1(1,2:4))])
disp(['s1:    end:   ', num2str(v_s1(end,2:4))])
disp(['cycle: start: ', num2str(v_cycle(1,2:4))])
disp(['cycle: end:   ', num2str(v_cycle(end,2:4))])
