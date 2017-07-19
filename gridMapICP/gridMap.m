clc
% clear
% Show the digital map
figure
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
% Load in dilated wall cloud
load('wallcloud_dilation.mat')

% Load in the trajectory of "add-only" strategy
% load('./trajectory/traj_grid_150604.mat')

% Show global map and trajectory
scatter(wallcloud(1,:),wallcloud(2,:),'filled','MarkerFaceColor','b','SizeData',3)
% scatter(traj_data(1,:),traj_data(2,:),'filled','MarkerFaceColor','g','SizeData',10)

% Load in LiDAR data
load('../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_32_16.mat')

step = 5;
dr = 1.0;
wr = 0.1;
visible = true;

% Initial pose
rotd1 = eul2rotm([deg2rad(0),0,0]);
% rotd1 = eul2rotm([deg2rad(-70),0,0]); 
% rotd1 = eul2rotm([deg2rad(-75),0,0]);   % 0726144311
% rotd1 = eul2rotm([deg2rad(-100),0,0]);  % 0726142910
rotd1 = rotd1(1:2,1:2);
trajd1 = [35.2;46.1];
% trajd1 = [37.2;46.1];   % 0726142826
% trajd1 = [75;29];   
% trajd1 = [75;31];       % 0726144311
% trajd1 = [60;34];       % 0726142910
% trajd1 = [77;43.5];   % 0801150651

wallcolor = 'r';
trajcolor = 'm';
bfd1 = [];
wc_U = [round(wallcloud*10)/10];  % Updated grid map
wc = round(wallcloud*10)/10;    % Original grid map
wc_update=[];
traj_data=[];
timetable=[];
ptnum=[];

it = 1;
t_all = tic;
for frame=1:step:(m/16)
    iter = 20;
    if frame~=m/16
        while xd1(frame, 1)==0
            frame = frame+1;
        end
    end

    tic
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

    x1_a = round(x1_a*10)/10;
    x2_a = round(x2_a*10)/10;
    x3_a = round(x3_a*10)/10;
    x4_a = round(x4_a*10)/10;
    y1_a = round(y1_a*10)/10;
    y2_a = round(y2_a*10)/10;
    y3_a = round(y3_a*10)/10;
    y4_a = round(y4_a*10)/10;

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
    t_6=toc;

    % Initial condition
    %% Original initial
    initRot = rotd1;
    initPos = trajd1;
    afd1tmp = afd1;

    tic    
    if size(wc_update,2) ~= 0
        wc_U = union(wc_U',round(wc_update'*10)/10,'rows')';
    end
    t_1=toc;

    tic
    afd1tmp(1:2,:)=initRot*afd1(1:2,:)+initPos*ones(1,size(afd1(1:2,:),2));
    [TRd1,TTd1]=icp(wc_U,afd1tmp,iter,'Matching','kDtree','WorstRejection',wr);
    initRot = TRd1(1:2,1:2)*initRot;
    initPos = TRd1(1:2,1:2)*initPos+TTd1(1:2,1);
    afd1tmp(1:2,:)=TRd1(1:2,1:2)*afd1tmp(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1tmp(1:2,:),2));
    [TRd1,TTd1,p_indxd1,q_indxd1]=icpMatch(wc_U,afd1tmp,iter,'Matching','kDtree',...
        'WorstRejection',dr,'UnmatchDistance',0.5);
    initRot = TRd1(1:2,1:2)*initRot;
    initPos = TRd1(1:2,1:2)*initPos+TTd1(1:2,1);
    afd1tmp(1:2,:)=TRd1(1:2,1:2)*afd1tmp(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1tmp(1:2,:),2));
    t_2=toc;

    %% 1st Moving point filter
    tic
    colid = ismembertol(round(afd1tmp'*10)/10,wc_dil','ByRows',true)';
    t_3=toc;
    movpoints = afd1(:,~colid);
    stapoints = afd1(:,colid);
    %% 2nd ICP
    initRot = rotd1;
    initPos = trajd1;
    afd1tmp = stapoints;

    tic
    afd1tmp(1:2,:)=initRot*afd1tmp(1:2,:)+initPos*ones(1,size(afd1tmp(1:2,:),2));
    afd1(1:2,:)=initRot*afd1(1:2,:)+initPos*ones(1,size(afd1(1:2,:),2));
    [TRd1,TTd1]=icp(wc,afd1tmp,iter,'Matching','kDtree','WorstRejection',wr);
    initRot = TRd1(1:2,1:2)*initRot;
    initPos = TRd1(1:2,1:2)*initPos+TTd1(1:2,1);
    afd1tmp(1:2,:)=TRd1(1:2,1:2)*afd1tmp(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1tmp(1:2,:),2));
    afd1(1:2,:)=TRd1(1:2,1:2)*afd1(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1(1:2,:),2));
    [TRd1,TTd1,p_indxd1,q_indxd1]=icpMatch(wc,afd1tmp,iter,'Matching','kDtree',...
        'WorstRejection',dr,'UnmatchDistance',0.5);
    initRot = TRd1(1:2,1:2)*initRot;
    initPos = TRd1(1:2,1:2)*initPos+TTd1(1:2,1);

    afd1(1:2,:)=TRd1(1:2,1:2)*afd1(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1(1:2,:),2));
    rotd1 = initRot;
    trajd1 = initPos;
    t_4=toc;

    %% 2nd Moving point filter
    tic
    colid = ismembertol(round(afd1'*10)/10,wc_dil','ByRows',true)';
    t_5=toc;
    movpoints = afd1(:,~colid);

    % Update map
    tic
    if it ~= 1
        wc_update=union(wc_update',round(movpoints'*10)/10,'rows')';
    else
        wc_update = round(movpoints*10)/10;
    end
    traj_data = [traj_data trajd1];
    ptnum = [ptnum size(wc_U,2)];
    it = it+1;
    preframe = frame;
    bf = af;
    bfd1 = afd1;
    bfd2 = afd2;
    t_7=toc;
    
    % Angle of view
    tic
    maxDist = max(max(v1_a),max(v2_a));
    oriAng = rotm2eul([rotd1 [0;0];0 0 0]);
    oriAng = oriAng(1);
    maxAng = max(max(d1_a),max(d2_a))+oriAng-pi/18;  % Radian
    minAng = min(min(d1_a),min(d2_a))+oriAng+pi/18;  % Radian
    [maskx,masky]=pol2cart([maxAng minAng],[maxDist maxDist]);
    t_8=toc;

    % Draw Map
    tic
    if visible
        hold on
        scatter(movpoints(1,:),movpoints(2,:),'filled','MarkerFaceColor','c','SizeData',3);
        wall=scatter(afd1(1,:),afd1(2,:),'filled','MarkerFaceColor',wallcolor,'SizeData',3);
        % Angle of view
        line1 = line([trajd1(1) trajd1(1)+maskx(1)], [trajd1(2) trajd1(2)+masky(1)]);
        line2 = line([trajd1(1) trajd1(1)+maskx(2)], [trajd1(2) trajd1(2)+masky(2)]);
        % Trajectory
        scatter(trajd1(1,:),trajd1(2,:),'filled','MarkerFaceColor',trajcolor,'SizeData',4);

        % xlim([0 size(A,2)/10])
        % ylim([0 size(A,1)/10])
        axis equal
        drawnow
    end
    disp(frame)
    t_9=toc;

    % if frame >730
    %     waitforbuttonpress;
    %     % break
    % end
    tic
    delete(wall)
    delete(line1)
    delete(line2)
    t_10=toc;

    timetable = [timetable;t_1 t_2 t_3 t_4 t_5 t_6 t_7 t_8 t_9 t_10];

end
% Save the wall point cloud
% save('wallcloud.mat','wallcloud','trajectory')
% save(savename(nameit,1:end),'wallcloud','traj_data')

if ~visible
    hold on
    scatter(wc_update(1,:),wc_update(2,:),'filled','MarkerFaceColor','c','SizeData',3);
    % Trajectory
    scatter(traj_data(1,:),traj_data(2,:),'filled','MarkerFaceColor',trajcolor,'SizeData',4);

    xlim([0 size(A,2)/10])
    ylim([0 size(A,1)/10])
    axis equal
    drawnow
end

toc(t_all)

disp('END')
cd ~/Project/gridMapICP
hold on

