clc
clear

load('./pose_graph/pose_20170523162221.mat')
v_bf = vertex;
e_bf = edges;

load('./g2o_optimizer/result_20170523162221.mat')
v_af = vertex;
e_af = edges;

% Origin
figure
hold on
scatter3(v_bf(:,2),v_bf(:,3),v_bf(:,4),5,'k','filled');
scatter3(v_af(:,2),v_af(:,3),v_af(:,4),5,'b','filled')
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')

% Rotate
figure
hold on
scatter3(v_bf(:,2),v_bf(:,3),v_bf(:,4),5,'k','filled');
scatter3(v_af(:,2),-v_af(:,3),-v_af(:,4),5,'b','filled')
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