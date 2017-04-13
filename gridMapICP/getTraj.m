clc
clear
% Show the digital map
A = imread('../Real_Map/utmMap.png');
% Grayscale
A(:,:,1)=A(:,:,2);
A(:,:,3)=A(:,:,2);
image([0,size(A,2)/10],[0,size(A,1)/10],flip(A,1),'AlphaData',0.5)
truesize
set(gca,'ydir','normal');
axis equal

hold on

% Load in data of wall point cloud and trajectory
load('wallcloud.mat')

% Read in CSV file and seperate the data
% GPS Latitude and Longitude
% gps = csvread('../read CPEV data/CPEV160801/CPEV_Record_2016_08_01_10_39_37_gps.csv');
% hdir = gps(:,1);    % Heading direction
% gps = gps(:,[3,4])/100;
% gps = floor(gps)+(gps-floor(gps))*100/60;
% [gps_x,gps_y,~]=deg2utm(gps(:,1),gps(:,2));
% gps = [gps_x,gps_y];

% Load in LiDAR data
load('../read CPEV data/CPEV160801/CPEV_Record_2016_08_01_15_02_21.mat')

step = 5;
dr = 1.0;
wr = 0.1;

% Initial pose
% gpsRot = eul2rotm([deg2rad(hdir(1)),0,0]);
% gpsRot = gpsRot(1:2,1:2);
% gps = gps*gpsRot';
rotd1 = eul2rotm([deg2rad(-3.6),0,0]);
rotd1 = rotd1(1:2,1:2);
trajd1 = [35.2;46.1];
% gps0 = gps(1,:);
% gpsTraj = trajd1';
wallcolor = 'r';
trajcolor = 'm';
bfd1 = [];
% wc = [wallcloud;zeros(1,size(wallcloud,2))];
wc = wallcloud;
scatter(wallcloud(1,:),wallcloud(2,:),'filled','MarkerFaceColor','b','SizeData',3)
scatter(trajectory(1,:),trajectory(2,:),'filled','MarkerFaceColor','g','SizeData',5)

it = 1;
for frame=1:step:(m/16)
    iter = 20;
    if frame~=m/16
        while xd1(frame, 1)==0
            frame = frame+1;
        end
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

    % Initial condition
    %% GPS initial
    % gpsTraj = gps(frame,:)-gps0+gpsTraj;
    % initRot = eul2rotm([-deg2rad(hdir(frame)-hdir(1)),0,0]);
    % initRot = initRot(1:2,1:2);
    % initPos = gpsTraj';
    %% Original initial
    initRot = rotd1;
    initPos = trajd1;

    afd1(1:2,:)=initRot*afd1(1:2,:)+initPos;
    [TRd1,TTd1] = icp(wc, afd1, iter, 'Matching', 'kDtree', 'WorstRejection', wr);
    rotd1 = TRd1(1:2,1:2)*initRot;
    trajd1 = TRd1(1:2,1:2)*initPos+TTd1(1:2,1);
    afd1(1:2,:)=TRd1(1:2,1:2)*afd1(1:2,:)+TTd1(1:2,1);
    [TRd1,TTd1] = icpMatch(wc, afd1, iter, 'Matching', 'kDtree', 'WorstRejection', dr);
    rotd1 = TRd1(1:2,1:2)*rotd1;
    trajd1 = TRd1(1:2,1:2)*trajd1+TTd1(1:2,1);
    afd1(1:2,:)=TRd1(1:2,1:2)*afd1(1:2,:)+TTd1(1:2,1);

    wall=scatter(afd1(1,:),afd1(2,:),'filled','MarkerFaceColor',wallcolor,'SizeData',3);
    hold on

    % Trajectory
    scatter(trajd1(1,:),trajd1(2,:),'filled','MarkerFaceColor',trajcolor,'SizeData',4)
    % scatter(gpsTraj(1),gpsTraj(2),'filled','MarkerFaceColor','y','SizeData',4)

    xlim([0 size(A,2)/10])
    ylim([0 size(A,1)/10])
    axis equal
    % disp(frame)
    drawnow



    it = it+1;
    preframe = frame;
    bf = af;
    bfd1 = afd1;
    bfd2 = afd2;
    % gps0 = gps(frame,:);
    % if frame >1000
    %     break
    % end
    delete(wall)


end
% Save the wall point cloud
% save('wallcloud.mat','wallcloud','trajectory')


disp('END')
cd ~/Dropbox/study/Project/gridMapICP
hold on

