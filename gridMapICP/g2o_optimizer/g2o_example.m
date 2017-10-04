clear
clc

pgfilename='../pose_graph/pose_example_trans_bf.mat';

edges(1,:) = [0 1 [1 1 0]+rand(1,3)*0.1-0.05 eul2quat([-pi/2 0 0]+rand(1,3)*pi/18-pi/36)];
edges(2,:) = [1 2 [1 0 0]+rand(1,3)*0.1-0.05 eul2quat([-pi/2 0 0]+rand(1,3)*pi/18-pi/36)];
edges(3,:) = [2 3 [1 0 0]+rand(1,3)*0.1-0.05 eul2quat([pi 0 0]+rand(1,3)*pi/18-pi/36)];
% edges(4,:) = [3 0 [0 0 0] eul2quat([0 0 0])];

vertex(1,:) = [0 [0 0 0] eul2quat([0 0 0])];
vertex(2,:) = [1 (quat2rotm(vertex(1,end-3:end))*edges(1,3:5)'+vertex(1,2:4)')' quatmultiply(vertex(1,end-3:end),edges(1,end-3:end))];
vertex(3,:) = [2 (quat2rotm(vertex(2,end-3:end))*edges(2,3:5)'+vertex(2,2:4)')' quatmultiply(vertex(2,end-3:end),edges(2,end-3:end))];
vertex(4,:) = [3 (quat2rotm(vertex(3,end-3:end))*edges(3,3:5)'+vertex(3,2:4)')' quatmultiply(vertex(3,end-3:end),edges(3,end-3:end))];



% pose = [0 0 0 eul2quat([0 0 0])];

% vertex(1,:) = [0, pose];
% edges(1,:) = [0 1 [1 1 0]+rand(1,3)*0.1-0.05 eul2quat([pi/4 0 0]+rand(1,3)*pi/18-pi/36)];
% pose(1:3) = (quat2rotm(edges(1,end-3:end))*pose(1:3)'+edges(1,3:5)')';
% pose(4:end) = quatmultiply(edges(1,end-3:end),pose(4:end));
% % pose(4:end) = rotm2quat(quat2rotm(edges(1,end-3:end))*quat2rotm(pose(4:end)));
% vertex(2,:) = [1,pose];

% edges(2,:) = [1 2 [1 1.4142 0]+rand(1,3)*0.1-0.05 eul2quat([-0.75*pi 0 0]+rand(1,3)*pi/18-pi/36)];
% pose(1:3) = (quat2rotm(edges(2,end-3:end))*pose(1:3)'+edges(2,3:5)')';
% pose(4:end) = quatmultiply(edges(2,end-3:end),pose(4:end));
% % pose(4:end) = rotm2quat(quat2rotm(edges(2,end-3:end))*quat2rotm(pose(4:end)));
% vertex(3,:) = [2,pose];

% edges(3,:) = [2 3 [0 1 0]+rand(1,3)*0.1-0.05 eul2quat([-0.5*pi 0 0]+rand(1,3)*pi/18-pi/36)];
% pose(1:3) = (quat2rotm(edges(3,end-3:end))*pose(1:3)'+edges(3,3:5)')';
% pose(4:end) = quatmultiply(edges(3,end-3:end),pose(4:end));
% % pose(4:end) = rotm2quat(quat2rotm(edges(3,end-3:end))*quat2rotm(pose(4:end)));
% vertex(4,:) = [3,pose];

% edges(4,:) = [3 0 0 0 0 eul2quat([pi 0 0])];

Edges = edges;

scatter3(vertex(:,2),vertex(:,3),vertex(:,4),10,'filled');
hold on
line(vertex(1:2,2),vertex(1:2,3),vertex(1:2,4))
line(vertex(2:3,2),vertex(2:3,3),vertex(2:3,4))
line(vertex(3:4,2),vertex(3:4,3),vertex(3:4,4))
% line(vertex([1,4],2),vertex([1,4],3),vertex([1,4],4),'Color','r')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-0.5 1.5])
ylim([-0.5 1.5])

% T0 = [quat2rotm(vertex(1,end-3:end)) vertex(1,2:4)';0 0 0 1];
% T1 = [quat2rotm(vertex(2,end-3:end)) vertex(2,2:4)';0 0 0 1];
% T2 = [quat2rotm(vertex(3,end-3:end)) vertex(3,2:4)';0 0 0 1];
% T3 = [quat2rotm(vertex(4,end-3:end)) vertex(4,2:4)';0 0 0 1];

% T01 = T0^-1*T1;
% T12 = T1^-1*T2;
% T23 = T2^-1*T3;

% edges(1,3:end) = [T01(1:3,end)' rotm2quat(T01(1:3,1:3))];
% edges(2,3:end) = [T12(1:3,end)' rotm2quat(T12(1:3,1:3))];
% edges(3,3:end) = [T23(1:3,end)' rotm2quat(T23(1:3,1:3))];

vertex(:,end-3:end) = [vertex(:,end-2:end) vertex(:,end-3)];
edges(:,end-3:end) = [edges(:,end-2:end) edges(:,end-3)];

save(pgfilename,'vertex','edges')
