clc
clear

pcdName  ='./pointcloud/20170913_1_f1s10_keypoints.mat';
load('../read CPEV data/CPEV170913/out.mat')

% Parameters
step = 10;
SearchRange = 2;
edge_th = 0.01;
frame = 1;

% Target keypoints
pc = raw(:,raw(1,:)==frame);
filtered_pc = pc_filter(pc,-0.5);
[x,y,z] = sph2cart(filtered_pc(3,:),filtered_pc(5,:),filtered_pc(4,:));
pc = [x;y;z];
% pc = pc(:,pc(1,:)>40 & pc(1,:)<80 & pc(2,:)>-50 & pc(2,:)<40);	% Frame 1
% pc = pc(:,pc(1,:)>-20 & pc(1,:)<20 & pc(2,:)>-10 & pc(2,:)<25);	% Frame 80

eigens = [];
eigenRatio = [];

for i=1:size(pc,2)
	pcSet = pc-pc(:,i);
	pcSet = pcSet(:,prod(abs(pcSet)<SearchRange,1)>0);
	eigens(:,i) = sort(eig(pcSet*pcSet'/size(pcSet,2)),'descend');
	eigenRatio(i) = sum(eigens(:,i))^3/prod(eigens(:,i));
end

surfvariation = eigens(3,:)./sum(eigens,1);
[~,surfID] = sort(surfvariation);

pt_surf = pc(:,surfvariation<edge_th);
pt_edge = pc(:,surfvariation>=edge_th);

top10ID = surfID(round(0.9*length(surfID)):end);
P_tar = pc(:,top10ID);

% Source keypoints
pc = raw(:,raw(1,:)==frame+step);
filtered_pc = pc_filter(pc,-0.5);
[x,y,z] = sph2cart(filtered_pc(3,:),filtered_pc(5,:),filtered_pc(4,:));
pc = [x;y;z];
% pc = pc(:,pc(1,:)>40 & pc(1,:)<80 & pc(2,:)>-50 & pc(2,:)<40);	% Frame 1
% pc = pc(:,pc(1,:)>-20 & pc(1,:)<20 & pc(2,:)>-10 & pc(2,:)<25);	% Frame 80

eigens = [];
eigenRatio = [];

for i=1:size(pc,2)
	pcSet = pc-pc(:,i);
	pcSet = pcSet(:,prod(abs(pcSet)<SearchRange,1)>0);
	eigens(:,i) = sort(eig(pcSet*pcSet'/size(pcSet,2)),'descend');
	eigenRatio(i) = sum(eigens(:,i))^3/prod(eigens(:,i));
end

surfvariation = eigens(3,:)./sum(eigens,1);
[~,surfID] = sort(surfvariation);

pt_surf = pc(:,surfvariation<edge_th);
pt_edge = pc(:,surfvariation>=edge_th);

top10ID = surfID(round(0.9*length(surfID)):end);
P_src_origin = pc(:,top10ID);

figure;hold on;axis equal;
scatter3(pc(1,:),pc(2,:),pc(3,:),5,'b','filled')

figure;hold on;axis equal;
scatter3(pc(1,surfID),pc(2,surfID),pc(3,surfID),5,linspace(1,10,size(surfvariation,2)),'filled')

figure;hold on;
plot(surfvariation,'.')
line([0 size(pc,2)],[edge_th edge_th],'Color','r')
xlabel('Point ID')
ylabel('Surface variation')

figure;hold on;axis equal;
scatter3(pt_surf(1,:),pt_surf(2,:),pt_surf(3,:),5,'k','filled')
scatter3(pt_edge(1,:),pt_edge(2,:),pt_edge(3,:),5,'r','filled')

figure;hold on;axis equal;
scatter3(pc(1,:),pc(2,:),pc(3,:),5,'k','filled')
scatter3(pc(1,top10ID),pc(2,top10ID),pc(3,top10ID),5,'r','filled')

% Save point cloud
P_tar = P_tar';
P_src_origin = P_src_origin';
save(pcdName,'P_tar','P_src_origin')