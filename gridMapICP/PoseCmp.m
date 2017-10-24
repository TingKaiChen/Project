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
scatter3(v_s1(:,2),v_s1(:,3),v_s1(:,4),PtSize,'k','filled');
% scatter3(v_s3(:,2),v_s3(:,3),v_s3(:,4),PtSize,'b','filled');
scatter3(v_cycle(:,2),v_cycle(:,3),v_cycle(:,4),PtSize,'r','filled')
axis equal
xlim([min([v_s1(:,2);v_s3(:,2);v_cycle(:,2)])-25, max([v_s1(:,2);v_s3(:,2);v_cycle(:,2)])+25])
ylim([min([v_s1(:,3);v_s3(:,3);v_cycle(:,3)])-25, max([v_s1(:,3);v_s3(:,3);v_cycle(:,3)])+25])
zlim([min([v_s1(:,4);v_s3(:,4);v_cycle(:,4)])-15, max([v_s1(:,4);v_s3(:,4);v_cycle(:,4)])+15])
xlabel('X')
ylabel('Y')
zlabel('Z')

% Calculate the difference
Distance_sq  = (v_cycle(:,2:4)-v_s1(:,2:4)).^2;
Distance     = sum(Distance_sq,2).^0.5;
Distance_Sum = sum(sum(Distance_sq,2).^0.5)
max(Distance)
mean(Distance)

se_dist_s1    = sum((v_s1(1,2:4)-v_s1(end,2:4)).^2).^0.5
% se_dist_s3    = sum((v_s3(1,2:4)-v_s3(end,2:4)).^2).^0.5
se_dist_cycle = sum((v_cycle(1,2:4)-v_cycle(end,2:4)).^2).^0.5

disp(['s1: 	  start: ', num2str(v_s1(1,2:4))])
disp(['s1:    end:   ', num2str(v_s1(end,2:4))])
% disp(['s3:    start: ', num2str(v_s1(1,2:4))])
% disp(['s3:    end:   ', num2str(v_s1(end,2:4))])
disp(['cycle: start: ', num2str(v_cycle(1,2:4))])
disp(['cycle: end:   ', num2str(v_cycle(end,2:4))])

% figure
% hold on
% plot(1:size(v_s1,1)-1,v_s1(2:end,2))
% plot(1:3:3*size(v_s3,1)-5,v_s3(2:end,2))
% plot(1:size(v_cycle,1)-1,v_cycle(2:end,2))
% title('pose X')
% xlabel('Frame')
% ylabel('(meter)')
% legend('step = 1','step = 3','optimized') 

% figure
% hold on
% plot(1:size(v_s1,1)-1,v_s1(2:end,3))
% plot(1:3:3*size(v_s3,1)-5,v_s3(2:end,3))
% plot(1:size(v_cycle,1)-1,v_cycle(2:end,3))
% title('pose Y')
% xlabel('Frame')
% ylabel('(meter)')
% legend('step = 1','step = 3','optimized') 

% figure
% hold on
% plot(1:size(v_s1,1)-1,v_s1(2:end,4))
% plot(1:3:3*size(v_s3,1)-5,v_s3(2:end,4))
% plot(1:size(v_cycle,1)-1,v_cycle(2:end,4))
% title('pose Z')
% xlabel('Frame')
% ylabel('(meter)')
% legend('step = 1','step = 3','optimized') 

% q_s1 = quat2eul([v_s1(:,end) v_s1(:,end-3:end-1)]);
% q_s3 = quat2eul([v_s3(:,end) v_s3(:,end-3:end-1)]);
% q_cycle = quat2eul(v_cycle(:,end-3:end));

% figure
% hold on
% plot(1:size(q_s1,1)-1,q_s1(2:end,1))
% plot(1:3:3*size(q_s3,1)-5,q_s3(2:end,1))
% plot(1:size(q_cycle,1)-1,q_cycle(2:end,1))
% title('pose Yaw')
% xlabel('Frame')
% ylabel('(rad)')
% legend('step = 1','step = 3','optimized') 

% figure
% hold on
% plot(1:size(q_s1,1)-1,q_s1(2:end,2))
% plot(1:3:3*size(q_s3,1)-5,q_s3(2:end,2))
% plot(1:size(q_cycle,1)-1,q_cycle(2:end,2))
% title('pose Roll')
% xlabel('Frame')
% ylabel('(rad)')
% legend('step = 1','step = 3','optimized') 

% figure
% hold on
% plot(1:size(q_s1,1)-1,q_s1(2:end,3))
% plot(1:3:3*size(q_s3,1)-5,q_s3(2:end,3))
% plot(1:size(q_cycle,1)-1,q_cycle(2:end,3))
% title('pose Pitch')
% xlabel('Frame')
% ylabel('(rad)')
% legend('step = 1','step = 3','optimized') 
