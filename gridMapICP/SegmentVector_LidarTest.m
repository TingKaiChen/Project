clc
clear

pcdName  ='./pointcloud/20170913_1_f80s10_keypoints_kpid.mat';
load('../read CPEV data/CPEV170913/out.mat')

% Parameters
step = 10;
r50 = 3;
edge_th = 0.01;
frame = 80;

% Target keypoints
pc = raw(:,raw(1,:)==frame);
filtered_pc = pc_filter(pc,-0.5);
[x,y,z] = sph2cart(filtered_pc(3,:),filtered_pc(5,:),filtered_pc(4,:));
pc = [x;y;z];
% pc = pc(:,pc(1,:)>40 & pc(1,:)<80 & pc(2,:)>-50 & pc(2,:)<40);	% Frame 1
% pc = pc(:,pc(1,:)>-20 & pc(1,:)<20 & pc(2,:)>-10 & pc(2,:)<25);	% Frame 80

eigens = [];
eigenRatio = [];

vecnorm = [];
segnum = [];
singval = [];
eigens = [];

for i=1:size(pc,2)
	d = norm(pc(:,i));
	pcSet = pc-pc(:,i);
	pcSet = pcSet(:,prod(abs(pcSet)<(r50*d/50),1)>0);
	centroid = mean(pcSet,2);
	X = pcSet;
	% X = pcSet-centroid;
	e = eig(X*X'/size(X,2));
	[V,D] = eig(X*X'/size(X,2),'vector');
	[~,ID] = min(D);

	vec(:,i) = -centroid;
	if -centroid'*V(:,ID) > 0
		vec(:,i) = V(:,ID);
	else
		vec(:,i) = -V(:,ID);
	end
	vec(:,i) = mean(vec(:,i)/norm(vec(:,i))-centroid/norm(centroid),2);
	vecnorm(i) = norm(vec(:,i));
	segnum(i) = sum(pcSet'*vec(:,i)<0)/size(pcSet,2);
	seg1 = sum(pcSet'*vec(:,i)<0)/size(pcSet,2);
	seg2 = sum(pcSet'*vec(:,i)>0)/size(pcSet,2);
	segnum(i) = 1-min(seg1,seg2)/max(seg1,seg2);
	eigens(i) = min(e)/sum(e);
	singval(i) = eigens(i)*segnum(i);
	% singval(i) = segnum(i);
end

[~,KPID] = sort(singval);

top10ID = KPID(round(0.9*length(KPID)):end);
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

vecnorm = [];
segnum = [];
singval = [];
eigens = [];

for i=1:size(pc,2)
	d = norm(pc(:,i));
	pcSet = pc-pc(:,i);
	pcSet = pcSet(:,prod(abs(pcSet)<(r50*d/50),1)>0);
	centroid = mean(pcSet,2);
	X = pcSet;
	% X = pcSet-centroid;
	e = eig(X*X'/size(X,2));
	[V,D] = eig(X*X'/size(X,2),'vector');
	[~,ID] = min(D);

	vec(:,i) = -centroid;
	if -centroid'*V(:,ID) > 0
		vec(:,i) = V(:,ID);
	else
		vec(:,i) = -V(:,ID);
	end
	vec(:,i) = mean(vec(:,i)/norm(vec(:,i))-centroid/norm(centroid),2);
	vecnorm(i) = norm(vec(:,i));
	segnum(i) = sum(pcSet'*vec(:,i)<0)/size(pcSet,2);
	seg1 = sum(pcSet'*vec(:,i)<0)/size(pcSet,2);
	seg2 = sum(pcSet'*vec(:,i)>0)/size(pcSet,2);
	segnum(i) = 1-min(seg1,seg2)/max(seg1,seg2);
	eigens(i) = min(e)/sum(e);
	singval(i) = eigens(i)*segnum(i);
	% singval(i) = segnum(i);
end

[~,KPID] = sort(singval);

top10ID = KPID(round(0.9*length(KPID)):end);
P_src_origin = pc(:,top10ID);

figure;hold on;axis equal;
scatter3(pc(1,:),pc(2,:),pc(3,:),5,'b','filled')

figure;hold on;axis equal;
scatter3(pc(1,KPID),pc(2,KPID),pc(3,KPID),5,jet(size(KPID,2)),'filled')

figure;hold on;
plot(singval,'.')
% line([0 size(pc,2)],[edge_th edge_th],'Color','r')
xlabel('Point ID')
ylabel('Keypoint Index')

% figure;hold on;axis equal;
% scatter3(pt_surf(1,:),pt_surf(2,:),pt_surf(3,:),5,'k','filled')
% scatter3(pt_edge(1,:),pt_edge(2,:),pt_edge(3,:),5,'r','filled')

figure;hold on;axis equal;
scatter3(pc(1,:),pc(2,:),pc(3,:),5,'k','filled')
scatter3(pc(1,top10ID),pc(2,top10ID),pc(3,top10ID),5,'r','filled')

% Save point cloud
P_tar = P_tar';
P_src_origin = P_src_origin';
save(pcdName,'P_tar','P_src_origin')