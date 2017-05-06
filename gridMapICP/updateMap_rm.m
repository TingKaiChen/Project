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

% Load in the trajectory of "add-only" strategy
load('./trajectory/traj_add_101017.mat')

% Load in LiDAR data
load('../read CPEV data/CPEV160801/CPEV_Record_2016_08_01_10_10_17.mat')

%Video
% outputV = VideoWriter('../figure/rm_101017_rad15');
% open(outputV)

step = 5;
dr = 1.0;
wr = 0.1;

% Initial pose
rotd1 = eul2rotm([deg2rad(-3.6),0,0]);
rotd1 = rotd1(1:2,1:2);
trajd1 = [35.2;46.1];
wallcolor = 'r';
trajcolor = 'm';
bfd1 = [];
% wc = [wallcloud;zeros(1,size(wallcloud,2))];
wc = wallcloud;
wc_U = wc;
wc_update=[];
wc_remove=[];
% scatter(wallcloud(1,:),wallcloud(2,:),'filled','MarkerFaceColor','b','SizeData',3)
scatter(traj_data(1,:),traj_data(2,:),'filled','MarkerFaceColor','g','SizeData',10)

it = 1;
for frame=1:step:(m/16)
    iter = 20;
    if frame~=m/16
        while xd1(frame, 1)==0
            frame = frame+1;
        end
    end
    if frame ~= 1
        delete(world)
        delete(upd)
        
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
    %% Original initial
    initRot = rotd1;
    initPos = trajd1;
    wc_U = [wc_U wc_update];

    afd1(1:2,:)=initRot*afd1(1:2,:)+initPos*ones(1,size(afd1(1:2,:),2));
    [TRd1,TTd1]=icp(wc_U,afd1,iter,'Matching','kDtree','WorstRejection',wr);
    rotd1 = TRd1(1:2,1:2)*initRot;
    trajd1 = TRd1(1:2,1:2)*initPos+TTd1(1:2,1);
    afd1(1:2,:)=TRd1(1:2,1:2)*afd1(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1(1:2,:),2));
    [TRd1,TTd1,p_indxd1,q_indxd1]=icpMatch(wc_U,afd1,iter,'Matching','kDtree',...
        'WorstRejection',dr,'UnmatchDistance',0.5);
    rotd1 = TRd1(1:2,1:2)*rotd1;
    trajd1 = TRd1(1:2,1:2)*trajd1+TTd1(1:2,1);
    afd1(1:2,:)=TRd1(1:2,1:2)*afd1(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1(1:2,:),2));

    % Mask
    %% Mask of distance
    wc_vec = wc_U-[trajd1;0]*ones(1,size(wc_U,2));
    [wc_rad,wc_rho] = cart2pol(wc_vec(1,:),wc_vec(2,:));
    maxDist = max(max(v1_a),max(v2_a));
    wc_mask = wc_U(:,wc_rho<maxDist);
    %% Mask of angle
    wc_rad = wc_rad(wc_rho<maxDist);
    wc_rho = wc_rho(wc_rho<maxDist);
    oriAng = rotm2eul([rotd1 [0;0];0 0 0]);
    oriAng = oriAng(1);
    maxAng = max(max(d1_a),max(d2_a))+oriAng-pi/18;  % Radian
    minAng = min(min(d1_a),min(d2_a))+oriAng+pi/18;  % Radian
    if maxAng > pi
        % wc_mask = wc_mask(:,(wc_rad>minAng | wc_rad<(maxAng-2*pi)));

        % wc_unirad = unique(wc_rad);
        % index = [];
        % for i=1:length(wc_unirad)
        %     index = [index find(wc_rad==wc_unirad(i),1)];
        % end
        % wc_mask = wc_mask(:,index);
        % index = (wc_rad(index)>minAng | wc_rad(index)<(maxAng-2*pi));
        % wc_mask = wc_mask(:,index);

        index = (wc_rad>minAng | wc_rad<(maxAng-2*pi));
        wc_mask = wc_mask(:,index);
        wc_rho = wc_rho(index);
        % [~,stindex]=sort(wc_rho);
        % wc_mask = wc_mask(:,stindex(1:floor(length(stindex)/10)));
        wc_mask = wc_mask(:,wc_rho<=15);
    elseif minAng < -pi
        % wc_mask = wc_mask(:,(wc_rad>(2*pi+minAng) | wc_rad<maxAng));

        % wc_unirad = unique(wc_rad);
        % index = [];
        % for i=1:length(wc_unirad)
        %     index = [index find(wc_rad==wc_unirad(i),1)];
        % end
        % wc_mask = wc_mask(:,index);
        % index = (wc_rad(index)>(2*pi+minAng) | wc_rad(index)<maxAng);
        % wc_mask = wc_mask(:,index);

        index = (wc_rad>(2*pi+minAng) | wc_rad<maxAng);
        wc_mask = wc_mask(:,index);
        wc_rho = wc_rho(index);
        % [~,stindex]=sort(wc_rho);
        % wc_mask = wc_mask(:,stindex(1:floor(length(stindex)/10)));
        wc_mask = wc_mask(:,wc_rho<=15);
    else
        % wc_mask = wc_mask(:,(wc_rad>minAng & wc_rad<maxAng));

        % wc_unirad = unique(wc_rad);
        % index = [];
        % for i=1:length(wc_unirad)
        %     index = [index find(wc_rad==wc_unirad(i),1)];
        % end
        % wc_mask = wc_mask(:,index);
        % index = (wc_rad(index)>minAng & wc_rad(index)<maxAng);
        % wc_mask = wc_mask(:,index);

        index = (wc_rad>minAng & wc_rad<maxAng);
        wc_mask = wc_mask(:,index);
        wc_rho = wc_rho(index);
        % [~,stindex]=sort(wc_rho);
        % wc_mask = wc_mask(:,stindex(1:floor(length(stindex)/10)));
        wc_mask = wc_mask(:,wc_rho<=15);


        % [x,y] = pol2cart(wc_rad(index),wc_rho(index));
        % wc_mask = [x;y;zeros(1,size(x,2))];
    end

    % Update map
    [~,~,pindex,~]=icpMatch(afd1,wc_mask,1,'Matching','kDtree','WorstRejection',dr,...
        'UnmatchDistance',0.3);
    % scatter(wc_mask(1,:),wc_mask(2,:),'filled','MarkerFaceColor','k','SizeData',5)
    wc_diff = wc_mask(:,pindex);
    wc_U = setdiff(wc_U',wc_diff','rows')';
    if frame ~= 1
        wc_update = setdiff(wc_update',wc_diff','rows')';
    end
    p = afd1(:,p_indxd1);
    wc_update=[wc_update p];
    
    world = scatter(wc_U(1,:),wc_U(2,:),'filled','MarkerFaceColor','b','SizeData',3);
    mask=scatter(wc_diff(1,:),wc_diff(2,:),'filled','MarkerFaceColor','k','SizeData',5);
    hold on
    upd=scatter(wc_update(1,:),wc_update(2,:),'filled','MarkerFaceColor','c','SizeData',3);
    hold on
    wall=scatter(afd1(1,:),afd1(2,:),'filled','MarkerFaceColor',wallcolor,'SizeData',3);
    hold on
    
    % Angle of view
    [maskx,masky]=pol2cart([maxAng minAng],[maxDist maxDist]);
    line1 = line([trajd1(1) trajd1(1)+maskx(1)], [trajd1(2) trajd1(2)+masky(1)]);
    hold on
    line2 = line([trajd1(1) trajd1(1)+maskx(2)], [trajd1(2) trajd1(2)+masky(2)]);
    hold on

    % Trajectory
    scatter(trajd1(1,:),trajd1(2,:),'filled','MarkerFaceColor',trajcolor,'SizeData',4)

    xlim([0 size(A,2)/10])
    ylim([0 size(A,1)/10])
    axis equal
    % disp(frame)
    drawnow

    for j = 1:5
        % writeVideo(outputV,getframe(gcf))
    end

    it = it+1;
    preframe = frame;
    bf = af;
    bfd1 = afd1;
    bfd2 = afd2;
    % if frame >1
    %     break
    % end
    % waitforbuttonpress;
    delete(wall)
    delete(mask)
    delete(line1)
    delete(line2)


end
% Save the wall point cloud
% save('wallcloud.mat','wallcloud','trajectory')

% writeVideo(outputV,getframe(gcf))
% close(outputV)


disp('END')
cd ~/Dropbox/study/Project/gridMapICP
hold on

