clc
clear
% Show the digital map
A = imread('../Real_Map/utmMap.png');
image([0,size(A,2)/10],[0,size(A,1)/10],flip(A,1))
truesize
set(gca,'ydir','normal');
axis equal

hold on

% Load in data of wall point cloud and trajectory
load('wallcloud.mat')
% wallcloud = repmat(wallcloud,1,10);

% Read in CSV file and seperate the data
% cd ~/Dropbox/study/Project/icp
data = csvread('../read CPEV data/CPEV160801/CPEV_Record_2016_08_01_10_39_37.csv');
[m, n] = size(data);
degmat = data(1:2:m, :);
val1 = data(2:16:m, :);
val2 = data(4:16:m, :);
val3 = data(6:16:m, :);
val4 = data(8:16:m, :);

% Set valmat and degmat to proper value
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

xd1 = [x1,x2];
yd1 = [y1,y2];
xd2 = [x3,x4];
yd2 = [y3,y4];
x = [x1,x2,x3,x4];
y = [y1,y2,y3,y4];

step = 5;
dr = 1.0;

% Initial pose
frame = 1;
it = (frame+4)/5;
iter = 20;
% rotd1 = [deg2rad(20) 0 0];
% rotd1 = eul2rotm(rotd1);
% rotd1 = rotd1(1:2,1:2);
rotd1 = [1 0;0 1];
locd1 = [35;46];
% locd1 = [0;0];


% vb = 'on';     % Visibility of trajectory All
% vbd1 = 'on';     % Visibility of trajectory L1+L2
% vbd2 = 'on';     % Visibility of trajectory L3+L4

for frame=1:step:(m/16)
	if frame~=m/16
	    while xd1(frame, 1)==0
	        frame = frame+1;
	    end
	end
	if frame > 201
		break
	end

	d1_a = deg1(frame,:);
	d2_a = deg2(frame,:);
	d3_a = deg3(frame,:);
	d4_a = deg4(frame,:);
	v1_a = val1(frame,:);
	v2_a = val2(frame,:);
	v3_a = val3(frame,:);
	v4_a = val4(frame,:);


	% Remove origins
	d1_a = d1_a(d1_a ~= pi/2);
	d2_a = d2_a(d2_a ~= pi/2);
	d3_a = d3_a(d3_a ~= pi/2);
	d4_a = d4_a(d4_a ~= pi/2);
	v1_a = v1_a(d1_a ~= pi/2);
	v2_a = v2_a(d2_a ~= pi/2);
	v3_a = v3_a(d3_a ~= pi/2);
	v4_a = v4_a(d4_a ~= pi/2);


	[x1_a, y1_a] = pol2cart(d1_a, v1_a);
	[x2_a, y2_a] = pol2cart(d2_a, v2_a);
	[x3_a, y3_a] = pol2cart(d3_a, v3_a);
	[x4_a, y4_a] = pol2cart(d4_a, v4_a);

	pt12_a = union([x1_a;y1_a]',[x2_a;y2_a]','rows')';
	xd1_a = pt12_a(1,:);
	yd1_a = pt12_a(2,:);
	pt34_a = union([x3_a;y3_a]',[x4_a;y4_a]','rows')';
	xd2_a = pt34_a(1,:);
	yd2_a = pt34_a(2,:);
	pt_a = union(pt12_a',pt34_a','rows')';
	x_a = pt_a(1,:);
	y_a = pt_a(2,:);

	% ICP
	af = [x_a;y_a;zeros(size(x_a))];
	afd1 = [xd1_a;yd1_a;zeros(size(xd1_a))];
	afd2 = [xd2_a;yd2_a;zeros(size(xd2_a))];
	wc = [wallcloud;zeros(1,size(wallcloud,2))];    % 3D representation of wallcloud

	afd1(1:2,:)=rotd1*afd1(1:2,:)+locd1;

	[TRd1,TTd1,~] = icp(wc, afd1, iter, 'Matching', 'kDtree', 'WorstRejection', 0.1);

	rotd1 = TRd1(1:2,1:2)*rotd1;
	locd1 = TRd1(1:2,1:2)*locd1+TTd1(1:2,1);
	obj = TRd1(1:2,1:2)*afd1(1:2,:)+TTd1(1:2);
	ob = [obj;zeros(1,size(obj,2))];

	[TRd1,TTd1,~] = icpMatch(wc, ob, iter, 'Matching', 'kDtree', 'WorstRejection', 1);

	rotd1 = TRd1(1:2,1:2)*rotd1;
	locd1 = TRd1(1:2,1:2)*locd1+TTd1(1:2,1);
	obj = TRd1(1:2,1:2)*obj+TTd1(1:2);

	% Point cloud
	scatter(wallcloud(1,:),wallcloud(2,:),'filled','MarkerFaceColor','b','SizeData',3)
	hold on
	sca = scatter(obj(1,:),obj(2,:),'filled','MarkerFaceColor','r','SizeData',3);
	hold on
	% Location
	scatter(locd1(1,:),locd1(2,:),'filled','MarkerFaceColor','g','SizeData',30)
	hold on
	redLoc = scatter(locd1(1,:),locd1(2,:),'filled','MarkerFaceColor','m','SizeData',30);
	hold on

	% xlim([0 size(A,2)/10])
	% ylim([0 size(A,1)/10])
	xlim([10 100])
    ylim([35 size(A,1)/10])
	% axis equal
	drawnow

	if frame == 201
		break
		% frame	% 71, 186
		% w = waitforbuttonpress;
	end

	delete(sca)
	delete(redLoc)
	it = it+1;
end

disp('END')
cd ~/Dropbox/study/Project/ManualMapICP
hold on

