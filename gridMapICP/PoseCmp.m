clc
clear

load('./pose_graph/pose_20170523162617.mat')
v_bf = vertex;
e_bf = edges;

load('./g2o_optimizer/result_20170523162617.mat')
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

figure
hold on
scatter3(v_bf(:,2),v_bf(:,3),v_bf(:,4),5,'k','filled');
scatter3(v_af(:,2),-v_af(:,3),-v_af(:,4),5,'b','filled')
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')