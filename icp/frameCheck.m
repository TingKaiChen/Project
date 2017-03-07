% Read in CSV file and seperate the data
data = csvread('../read CPEV data/CPEV_Record_2016_08_01_10_10_17.csv');
[m, n] = size(data);
degmat = data(1:2:m, :);
val1 = data(2:16:m, :);
val2 = data(4:16:m, :);
val3 = data(6:16:m, :);
val4 = data(8:16:m, :);

% Set valmat and degmat to proper value
% degmat = degmat./(180^2).*2.*(pi^2);  % Radians
degmat = degmat./(5760).*(pi)+(pi/2);  % Radians
deg1 = degmat(1:8:m/2, :);
deg2 = degmat(2:8:m/2, :);
deg3 = degmat(3:8:m/2, :);
deg4 = degmat(4:8:m/2, :);
val1 = val1/100;
val2 = val2/100;
val3 = val3/100;
val4 = val4/100;

[x1, y1] = pol2cart(deg1, val1);
[x2, y2] = pol2cart(deg2, val2);
[x3, y3] = pol2cart(deg3, val3);
[x4, y4] = pol2cart(deg4, val4);

x = x4;
y = y4;

% Video
% outputV = VideoWriter('v2', 'MPEG-4');
% open(outputV)

figure('Name','LiDAR ICP','Position',[100 100 1400 900])
subplot(2,2,1)
scatter(x(1,:),y(1,:),'filled','MarkerFaceColor','k');
xlim([-20 20])
ylim([0 40])

subplot(2,2,3)
traj = [0;0];
scatter(traj(1),traj(2),'filled','MarkerFaceColor','b');
hold on

% writeVideo(outputV,getframe(gcf))
step = 5;
frame = 96;
pre = 91;
rot = [1 0;0 1];
check = false;
nz = sum(x(frame,:)~=0);
figure
af = [x(frame,1:nz);y(frame,1:nz);zeros(1,nz)];
bf = [x(pre,1:nz);y(pre,1:nz);zeros(1,nz)];
[TR, TT, ER, t] = icp(af, bf, 10000, 'Matching', 'kDtree');

% if ER(iter+1) > 0.5
%     check = true;
%     [TR, TT, ER, t] = icp(af, bf, 1000, 'Matching', 'kDtree');
% end
if check == true
    disp(['frame: ', num2str(frame), ' ER: ', num2str(ER(iter+1))]);
end

newXY = TR(1:2,1:2)*bf(1:2,:)+TT(1:2,1)*ones(1,nz);
scatter(x(frame, :),y(frame,:),'filled','MarkerFaceColor','r');
hold on
scatter(newXY(1, :),newXY(2,:),'MarkerEdgeColor','k');
hold off
xlim([-20 20])
ylim([0 40])

%%
% Read in CSV file and seperate the data
data = csvread('../read CPEV data/CPEV_Record_2016_08_01_10_10_17.csv');
[m, n] = size(data);
degmat = data(1:2:m, :);
val1 = data(2:16:m, :);
val2 = data(4:16:m, :);
val3 = data(6:16:m, :);
val4 = data(8:16:m, :);

% Set valmat and degmat to proper value
% degmat = degmat./(180^2).*2.*(pi^2);  % Radians
degmat = degmat./(5760).*(pi)+(pi/2);  % Radians
deg1 = degmat(1:8:m/2, :);
deg2 = degmat(2:8:m/2, :);
deg3 = degmat(3:8:m/2, :);
deg4 = degmat(4:8:m/2, :);
val1 = val1/100;
val2 = val2/100;
val3 = val3/100;
val4 = val4/100;

[x1, y1] = pol2cart(deg1, val1);
[x2, y2] = pol2cart(deg2, val2);
[x3, y3] = pol2cart(deg3, val3);
[x4, y4] = pol2cart(deg4, val4);

% Video
% outputV = VideoWriter('v2', 'MPEG-4');
% open(outputV)

frame = 1;
figure(1)
scatter(x1(frame,:),y1(frame,:),'filled','MarkerFaceColor','r');
hold on
scatter(x2(frame,:),y2(frame,:),'filled','MarkerFaceColor','y');
hold on
scatter(x3(frame,:),y3(frame,:),'filled','MarkerFaceColor','g');
hold on
scatter(x4(frame,:),y4(frame,:),'filled','MarkerFaceColor','b');
hold off
xlim([-20 20])
ylim([0 40])
axis equal

