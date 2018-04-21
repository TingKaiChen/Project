clc
clear

% Parameters
% edge_th = 0.001;	% noise = 0
% edge_th = 0.015;	% noise = 2
edge_th = 0.05;		% noise = 5

% % Generate a random plane (ax+by+cz=1)
plane1 = [3;2;4];
pc1 = [rand(400,2)*100-50];
pc1 = [pc1,(1-(pc1*plane1(1:2,1)))/plane1(3)];
% plane2 = [1;1;-1];
plane2 = [3;3;1];
pc2 = [rand(400,2)*100-50];
pc2 = [pc2,(1-(pc2*plane2(1:2,1)))/plane2(3)];
pc1 = pc1(pc1*plane2>=1,:)';
pc2 = pc2(pc2*plane1>=1,:)';
pc = [pc1 pc2];
pc1ID = 1:size(pc1,2);
pc2ID = (size(pc1,2)+1):size(pc,2);
noise = rand(size(pc))*10-5;
pc = pc+noise;

eigens = [];
eigenRatio = [];

for i=1:size(pc,2)
	pcSet = pc-pc(:,i);
	pcSet = pcSet(:,prod(abs(pcSet)<20,1)>0);
	eigens(:,i) = sort(eig(pcSet*pcSet'/size(pcSet,2)),'descend');
	eigenRatio(i) = sum(eigens(:,i))^3/prod(eigens(:,i));
end

surfvariation = eigens(3,:)./sum(eigens,1);
[~,surfID] = sort(surfvariation);

pt_surf = pc(:,surfvariation<edge_th);
pt_edge = pc(:,surfvariation>=edge_th);

top10ID = surfID(round(0.9*length(surfID)):end);


% eigens(2,eigens(2,:)<=0) = 0.1;
% t = eigens(1,:)./eigens(2,:);
% c = linspace(0,10,size(t,2));
% [~,I] = sort(t,'descend');

figure;hold on;
scatter3(pc(1,pc1ID),pc(2,pc1ID),pc(3,pc1ID),5,'r','filled')
scatter3(pc(1,pc2ID),pc(2,pc2ID),pc(3,pc2ID),5,'b','filled')

figure;hold on;
scatter3(pc(1,surfID),pc(2,surfID),pc(3,surfID),5,linspace(1,10,size(surfvariation,2)),'filled')

figure;hold on;
plot(surfvariation,'.')
line([0 size(pc,2)],[edge_th edge_th],'Color','r')
xlabel('Point ID')
ylabel('Surface variation')

figure;hold on;
scatter3(pt_surf(1,:),pt_surf(2,:),pt_surf(3,:),5,'m','filled')
scatter3(pt_edge(1,:),pt_edge(2,:),pt_edge(3,:),5,'g','filled')

figure;hold on;
scatter3(pc(1,:),pc(2,:),pc(3,:),5,'m','filled')
scatter3(pc(1,top10ID),pc(2,top10ID),pc(3,top10ID),5,'g','filled')

% scatter3(pc(1,t>2),pc(2,t>2),pc(3,t>2),10,'g','filled')
% figure
% plot(t)
% figure;hold on;
% scatter3(pc(1,:),pc(2,:),pc(3,:),5,c(I),'filled')

% figure
% hold on
% scatter3(pc(1,:),pc(2,:),pc(3,:),5,eigenRatio,'filled')
