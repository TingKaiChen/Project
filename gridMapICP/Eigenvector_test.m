clc
clear

% Parameters
searchRange = 20;
ptNum = 600;
noise = 5;

% % Generate 3 planes (x=0, y=0 & z=0)
% plane1 = [1;0;0];
plane1 = [1;-1;0];
% plane1 = [1;1;0];
pc1 = [zeros(1,ptNum);rand(2,ptNum)*100];
pc1(1,:) = 0-plane1(2)*pc1(2,:);
plane2 = [0;1;0];
pc2 = [rand(1,ptNum)*100;zeros(1,ptNum);rand(1,ptNum)*100];
plane3 = [0;0;1];
pc3 = [rand(1,ptNum)*200-100;rand(1,ptNum)*100;zeros(1,ptNum)];
pc3 = pc3(:,(plane1'*pc3)>=0);
ed = zeros(3,300);
ed(1,1:100) = rand(1,100)*100;
ed(2,101:200) = rand(1,100)*100;
ed(1,101:200) = 0-plane1(2)*ed(2,101:200);
ed(3,201:300) = rand(1,100)*100;
pc = [pc1 pc2 pc3 ed];
noisemat = (rand(size(pc))*2-1)*noise;
pc = pc+noisemat;
pc1ID = 1:size(pc1,2);
pc2ID = (size(pc1,2)+1):(size(pc1,2)+size(pc2,2));
pc3ID = (size(pc1,2)+size(pc2,2)+1):(size(pc1,2)+size(pc2,2)+size(pc3,2));
edID = (size(pc1,2)+size(pc2,2)+size(pc3,2)+1):size(pc,2);


% Eigenvector calculation
ev1 = [];	% Largest
ev2 = [];	% Midium
ev3 = [];	% Smallest
vec = [];
vecnorm = [];
segnum = [];
singval = [];
eigens = [];
for i=1:size(pc,2)
	% pcSet = pc-pc(:,i);
	% pcSet = pcSet(:,prod(abs(pcSet)<searchRange,1)>0);
	% [V,D] = eig(pcSet*pcSet'/size(pcSet,2),'vector');
	% [~,eigenID] = sort(D','descend');
	% ev1(:,i) = V(:,eigenID(1));
	% ev2(:,i) = V(:,eigenID(2));
	% ev3(:,i) = V(:,eigenID(3));

	pcSet = pc-pc(:,i);
	pcSet = pcSet(:,prod(abs(pcSet)<searchRange,1)>0);
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
end
[~,normID] = sort(vecnorm);
[~,segID] = sort(segnum);
[~,singID] = sort(singval);
[~,surfID] = sort(eigens);

% figure;hold on;
% scatter3(pc(1,pc1ID),pc(2,pc1ID),pc(3,pc1ID),5,'r','filled')
% scatter3(pc(1,pc2ID),pc(2,pc2ID),pc(3,pc2ID),5,'b','filled')
% scatter3(pc(1,pc3ID),pc(2,pc3ID),pc(3,pc3ID),5,'m','filled')
% scatter3(pc(1,edID),pc(2,edID),pc(3,edID),5,'c','filled')
% % quiver3(pc(1,edID),pc(2,edID),pc(3,edID),ev3(1,edID),ev3(2,edID),ev3(3,edID));
% quiver3(pc(1,:),pc(2,:),pc(3,:),vec(1,:),vec(2,:),vec(3,:),3);
% axis auto;

figure;hold on;
scatter3(pc(1,pc1ID),pc(2,pc1ID),pc(3,pc1ID),5,'r','filled')
scatter3(pc(1,pc2ID),pc(2,pc2ID),pc(3,pc2ID),5,'b','filled')
scatter3(pc(1,pc3ID),pc(2,pc3ID),pc(3,pc3ID),5,'m','filled')
scatter3(pc(1,edID),pc(2,edID),pc(3,edID),5,'c','filled')
axis equal
title('Surfaces and Edge of Point Cloud')

figure('Position',[77,393,864 422]);hold on;
subplot(1,2,1)
scatter3(pc(1,normID),pc(2,normID),pc(3,normID),5,jet(size(pc,2)),'filled')
% quiver3(pc(1,:),pc(2,:),pc(3,:),vec(1,:),vec(2,:),vec(3,:),3);
axis equal
subplot(1,2,2)
plot(sort(vecnorm))
xlabel('Point No.')
ylabel('Centroid Vector Norm')
suptitle('Centroid Vector Norm')

figure('Position',[77,393,864 422]);hold on;
subplot(1,2,1)
scatter3(pc(1,segID),pc(2,segID),pc(3,segID),5,jet(size(pc,2)),'filled')
axis equal
subplot(1,2,2)
plot(sort(segnum))
xlabel('Point No.')
ylabel('Segment Ratio')
suptitle('Segment Ratio')

figure('Position',[77,393,864 422]);hold on;
subplot(1,2,1)
scatter3(pc(1,surfID),pc(2,surfID),pc(3,surfID),5,jet(size(pc,2)),'filled')
axis equal
subplot(1,2,2)
plot(sort(eigens))
xlabel('Point No.')
ylabel('Surface Variation')
suptitle('Surface Variation')

figure('Position',[77,393,864 422]);hold on;
subplot(1,2,1)
scatter3(pc(1,singID),pc(2,singID),pc(3,singID),5,jet(size(pc,2)),'filled')
axis equal
subplot(1,2,2)
plot(sort(singval))
xlabel('Point No.')
ylabel('Keypoint Index')
suptitle('Keypoint Index')
