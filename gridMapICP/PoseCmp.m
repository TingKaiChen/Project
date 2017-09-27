clc
clear

load('./pose_graph/pose_example_quatfix_fix_af.mat')
v_bf = vertex;
e_bf = edges;

load('./g2o_optimizer/result_example_quatfix_fix_af.mat')
v_af = vertex;
e_af = edges;

% Origin
figure
hold on
scatter3(v_bf(:,2),v_bf(:,3),v_bf(:,4),[5:(30-5+1)/size(v_bf,1):30],'k','filled');
scatter3(v_af(:,2),v_af(:,3),v_af(:,4),[5:(30-5+1)/size(v_af,1):30],'r','filled')
line(v_bf([1,2],2),v_bf([1,2],3),v_bf([1,2],4));
line(v_bf([2,3],2),v_bf([2,3],3),v_bf([2,3],4));
line(v_bf([3,4],2),v_bf([3,4],3),v_bf([3,4],4));
% line(v_bf([4,1],2),v_bf([4,1],3),v_bf([4,1],4));
line(v_af([1,2],2),v_af([1,2],3),v_af([1,2],4),'Color','r');
line(v_af([2,3],2),v_af([2,3],3),v_af([2,3],4),'Color','r');
line(v_af([3,4],2),v_af([3,4],3),v_af([3,4],4),'Color','r');
% line(v_af([4,1],2),v_af([4,1],3),v_af([4,1],4),'Color','r');
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')

% % Rotate
% figure
% hold on
% scatter3(v_bf(:,2),v_bf(:,3),v_bf(:,4),[5:(30-5+1)/size(v_bf,1):30],'k','filled');
% scatter3(v_af(:,2),-v_af(:,3),-v_af(:,4),[5:(30-5+1)/size(v_af,1):30],'r','filled')
% line(v_bf([1,2],2),v_bf([1,2],3),v_bf([1,2],4));
% line(v_bf([2,3],2),v_bf([2,3],3),v_bf([2,3],4));
% line(v_bf([3,4],2),v_bf([3,4],3),v_bf([3,4],4));
% % line(v_bf([4,1],2),v_bf([4,1],3),v_bf([4,1],4));
% line(v_af([1,2],2),-v_af([1,2],3),-v_af([1,2],4),'Color','r');
% line(v_af([2,3],2),-v_af([2,3],3),-v_af([2,3],4),'Color','r');
% line(v_af([3,4],2),-v_af([3,4],3),-v_af([3,4],4),'Color','r');
% % line(v_af([4,1],2),-v_af([4,1],3),-v_af([4,1],4),'Color','r');
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')

% Calculate the difference
v_af_rot = [v_af(:,2),-v_af(:,3),-v_af(:,4)];
Distance_sq = (v_af_rot-v_bf(:,2:4)).^2;
Distance = sum(Distance_sq,2).^0.5;
Distance_Sum = sum(sum(Distance_sq,2).^0.5)
max(Distance)
mean(Distance)