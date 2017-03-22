clc
clear
% Show the digital map
A = imread('../Real_Map/utmMap.png');
image([0,size(A,2)/10],[0,size(A,1)/10],flip(A,1))
truesize
set(gca,'ydir','normal');
axis equal

hold on

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
rotd1 = eul2rotm([deg2rad(-3.6),0,0]);
rotd1 = rotd1(1:2,1:2);
trajd1 = [35.2;46.1];
wallcloud = [];
wallcolor = 'b';
trajcolor = 'g';
bfd1 = [];
trajectory = [];

vb = 'on';     % Visibility of trajectory All
vbd1 = 'on';     % Visibility of trajectory L1+L2
vbd2 = 'on';     % Visibility of trajectory L3+L4

it = 1;
for frame=1:step:(m/16)
    iter = 20;
    if frame~=m/16
        while xd1(frame, 1)==0
            frame = frame+1;
        end
    end
    if frame >201
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

    if frame ~= 1
        [TRd1,TTd1,q_idxd1] = icpMatch(afd1, bfd1, iter, 'Matching', 'kDtree', 'WorstRejection', dr);

        rotd1 = TRd1(1:2,1:2)'*rotd1;
        trajd1 = trajd1-rotd1*TTd1(1:2,1);
        trajectory = [trajectory trajd1];

        % Find the unmatched points and add them into wall clouds
        qid = unique(q_idxd1);
        qx = afd1(1,:);
        qy = afd1(2,:);
        qx(qid)=[];
        qy(qid)=[];
        q = [qx;qy];
        q = rotd1*q+trajd1;
        % scatter(qx,qy,'filled','MarkerFaceColor','b','SizeData',3)
        wallcloud = [wallcloud q];
        scatter(q(1,:),q(2,:),'filled','MarkerFaceColor',wallcolor,'SizeData',3)
        hold on
    else
        wallcloud = rotd1*afd1(1:2,:)+trajd1;
        scatter(wallcloud(1,:),wallcloud(2,:),'filled','MarkerFaceColor',wallcolor,'SizeData',3)
        hold on
    end 

    % Trajectory
    scatter(trajd1(1,:),trajd1(2,:),'filled','MarkerFaceColor',trajcolor,'SizeData',3)

    xlim([0 size(A,2)/10])
    ylim([0 size(A,1)/10])
    axis equal
    drawnow



    it = it+1;
    preframe = frame;
    bf = af;
    bfd1 = afd1;
    bfd2 = afd2;


end
% Save the wall point cloud
save('wallcloud.mat','wallcloud','trajectory')



disp('END')
cd ~/Dropbox/study/Project/ManualMapICP
hold on

