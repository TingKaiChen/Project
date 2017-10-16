clc
clear

load('./pose_graph/ICPcycle_20170523162617.mat')
v_bf = vertex;
e_bf = edges;

load('./g2o_optimizer/result_ICPcycle_20170523162617.mat')
v_af = vertex;
e_af = edges;

% Origin
figure
hold on
% PtSize_b = [5:(30-5)/(size(v_bf,1)-1):30];
PtSize_b = 5;
% PtSize_a = [5:(30-5)/(size(v_af,1)-1):30];
PtSize_a = 5;
scatter3(v_bf(:,2),v_bf(:,3),v_bf(:,4),PtSize_b,'k','filled');
scatter3(v_af(:,2),v_af(:,3),v_af(:,4),PtSize_a,'r','filled')

% % Line for example
% line(v_bf([1,2],2),v_bf([1,2],3),v_bf([1,2],4));
% line(v_bf([2,3],2),v_bf([2,3],3),v_bf([2,3],4));
% line(v_bf([3,4],2),v_bf([3,4],3),v_bf([3,4],4));
% line(v_bf([4,1],2),v_bf([4,1],3),v_bf([4,1],4));
% line(v_af([1,2],2),v_af([1,2],3),v_af([1,2],4),'Color','r');
% line(v_af([2,3],2),v_af([2,3],3),v_af([2,3],4),'Color','r');
% line(v_af([3,4],2),v_af([3,4],3),v_af([3,4],4),'Color','r');
% line(v_af([4,1],2),v_af([4,1],3),v_af([4,1],4),'Color','r');
axis equal
xlim([min([v_bf(:,2);v_af(:,2)])-25, max([v_bf(:,2);v_af(:,2)])+25])
ylim([min([v_bf(:,3);v_af(:,3)])-25, max([v_bf(:,3);v_af(:,3)])+25])
zlim([min([v_bf(:,4);v_af(:,4)])-15, max([v_bf(:,4);v_af(:,4)])+15])
xlabel('X')
ylabel('Y')
zlabel('Z')

% Calculate the difference
Distance_sq = (v_af(:,2:4)-v_bf(:,2:4)).^2;
Distance = sum(Distance_sq,2).^0.5;
Distance_Sum = sum(sum(Distance_sq,2).^0.5)
max(Distance)
mean(Distance)

se_dist_b = sum((v_bf(1,2:4)-v_bf(end,2:4)).^2).^0.5
se_dist_a = sum((v_af(1,2:4)-v_af(end,2:4)).^2).^0.5

disp(['bf: start: ', num2str(v_bf(1,2:4))])
disp(['bf: end:   ', num2str(v_bf(end,2:4))])
disp(['af: start: ', num2str(v_af(1,2:4))])
disp(['af: end:   ', num2str(v_af(end,2:4))])