clc
clear
figure
% Show the digital map
% A = imread('../Real_Map/utmMap.png');
% % Grayscale
% A(:,:,1)=A(:,:,2);
% A(:,:,3)=A(:,:,2);
% image([0,size(A,2)/10],[0,size(A,1)/10],flip(A,1),'AlphaData',0.5)
% truesize
% set(gca,'ydir','normal');
axis equal

hold on

% Read in CSV file and seperate the data
csvfilename='../read CPEV data/CPEV170523/CPEV_Record_2017_05_23_16_22_21.mat';
mapfilename='./globalmap/wallcloud_20170522132724.mat';
% % cd ~/Dropbox/study/Project/icp
% data = csvread(csvfilename);
% [m, n] = size(data);
% degmat = data(1:2:m, :);
% val1 = data(2:16:m, :);
% val2 = data(4:16:m, :);
% val3 = data(6:16:m, :);
% val4 = data(8:16:m, :);

% % Set valmat and degmat to proper value
% degmat = degmat./(5760).*(pi)+(pi/2);  % Radians
% deg1 = degmat(1:8:m/2, :);
% deg2 = degmat(2:8:m/2, :);
% deg3 = degmat(3:8:m/2, :);
% deg4 = degmat(4:8:m/2, :);
% val1 = val1/100;
% val2 = val2/100;
% val3 = val3/100;
% val4 = val4/100;

% [x1, y1] = pol2cart(deg1, val1);
% [x2, y2] = pol2cart(deg2, val2);
% [x3, y3] = pol2cart(deg3, val3);
% [x4, y4] = pol2cart(deg4, val4);

% xd1 = [x1,x2];
% yd1 = [y1,y2];
% xd2 = [x3,x4];
% yd2 = [y3,y4];
% x = [x1,x2,x3,x4];
% y = [y1,y2,y3,y4];

load('../read CPEV data/CPEV170523/CPEV_Record_2017_05_23_16_26_17.mat')

step = 1;
dr = 1.0;

% Initial pose
rotd1 = eul2rotm([deg2rad(-3.6),0,0]);
rotd1 = rotd1(1:2,1:2);
trajd1 = [35.2;46.1];
wallcloud = [];
wallcolor = 'b';
trajcolor = 'k';
bfd1 = [];
trajectory = [];

tic
it = 1;
for frame=1:step:(m/16)
    iter = 50;
    if frame~=m/16
        while xd1(frame, 1)==0
            frame = frame+1;
        end
    end
    % if frame >201
    %     break
    % end

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
        initRot = rotd1;
        initPos = trajd1;
        afd1tmp = afd1;
        % Initial condition

        afd1tmp(1:2,:)=initRot*afd1tmp(1:2,:)+initPos*ones(1,size(afd1tmp(1:2,:),2));
        afd1(1:2,:)=initRot*afd1(1:2,:)+initPos*ones(1,size(afd1(1:2,:),2));
        [TRd1,TTd1]=icp(wallcloud,afd1tmp,iter,'Matching','kDtree','WorstRejection',0.1);
        initRot = TRd1(1:2,1:2)*initRot;
        initPos = TRd1(1:2,1:2)*initPos+TTd1(1:2,1);
        afd1tmp(1:2,:)=TRd1(1:2,1:2)*afd1tmp(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1tmp(1:2,:),2));
        afd1(1:2,:)=TRd1(1:2,1:2)*afd1(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1(1:2,:),2));
        [TRd1,TTd1,p_indxd1,q_indxd1]=icpMatch(wallcloud,afd1tmp,iter,'Matching','kDtree',...
            'WorstRejection',dr,'UnmatchDistance',0.5);
        initRot = TRd1(1:2,1:2)*initRot;
        initPos = TRd1(1:2,1:2)*initPos+TTd1(1:2,1);

        afd1(1:2,:)=TRd1(1:2,1:2)*afd1(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1(1:2,:),2));
        rotd1 = initRot;
        trajd1 = initPos;

        % 
        % afd1(1:2,:)=rotd1*afd1(1:2,:)+trajd1;
        % [TRd1,TTd1,p_idxd1] = icpMatch(wallcloud, afd1, iter, 'Matching', 'kDtree', 'WorstRejection', dr);

        % rotd1 = TRd1(1:2,1:2)*rotd1;
        % trajd1 = TRd1(1:2,1:2)*trajd1+TTd1(1:2,1);
        trajectory = [trajectory trajd1];

        p = afd1(:,p_indxd1);
        % p = TRd1*p+TTd1;
        % afd1(1:2,:)=TRd1(1:2,1:2)*afd1(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1(1:2,:),2));
        % scatter(px,py,'filled','MarkerFaceColor','b','SizeData',3)
        wallcloud = union(wallcloud',round(p'*10)/10,'rows')';
        % scatter(p(1,:),p(2,:),'filled','MarkerFaceColor',wallcolor,'SizeData',3);
        wc.XData=[wc.XData p(1,:)];
        wc.YData=[wc.YData p(2,:)];
        wc.CData=linspace(1,10,length(wc.XData));
        traj.XData=[traj.XData trajd1(1,:)];
        traj.YData=[traj.YData trajd1(2,:)];
        hold on
    else
        wallcloud = rotd1*afd1(1:2,:)+trajd1;
        wallcloud = [round(wallcloud*10)/10;zeros(1,size(wallcloud,2))];
        wc=scatter(wallcloud(1,:),wallcloud(2,:),3,linspace(1,10,size(wallcloud,2)),'filled');

        % Trajectory
        traj = scatter(trajd1(1,:),trajd1(2,:),3,trajcolor,'filled');

        hold on
    end 


    % Angle of view
    maxDist = max(max(v1_a),max(v2_a));
    oriAng = rotm2eul([rotd1 [0;0];0 0 0]);
    oriAng = oriAng(1);
    maxAng = max(max(d1_a),max(d2_a))+oriAng-pi/18;  % Radian
    minAng = min(min(d1_a),min(d2_a))+oriAng+pi/18;  % Radian
    [maskx,masky]=pol2cart([maxAng minAng],[maxDist maxDist]);

    wall=scatter(afd1(1,:),afd1(2,:),3,'r','filled');
    % Angle of view
    line1 = line([trajd1(1) trajd1(1)+maskx(1)], [trajd1(2) trajd1(2)+masky(1)]);
    line2 = line([trajd1(1) trajd1(1)+maskx(2)], [trajd1(2) trajd1(2)+masky(2)]);


    % xlim([0 size(A,2)/10])
    % ylim([0 size(A,1)/10])
    axis equal
    % drawnow

    disp(frame)
    % waitforbuttonpress;


    it = it+1;
    preframe = frame;
    bf = af;
    bfd1 = afd1;
    bfd2 = afd2;


    delete(wall)
    delete(line1)
    delete(line2)
end
% Save the wall point cloud
% save(mapfilename,'wallcloud','trajectory')

disp('END')
% cd ~/Dropbox/study/Project/gridMapICP
hold on

toc