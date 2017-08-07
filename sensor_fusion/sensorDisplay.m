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
load('141303.mat')
trajpoint = traj_data;

scatter(wallcloud(1,:),wallcloud(2,:),'filled','MarkerFaceColor','b','SizeData',3)
scatter(traj_data(1,:),traj_data(2,:),'filled','MarkerFaceColor','g','SizeData',5)

% Load in LiDAR data
load('../read CPEV data/CPEV160801/CPEV_Record_2016_08_01_14_13_03.mat')

step = 5;
dr = 1.0;
wr = 0.1;

% Initial pose
rotarr = [];
rotd1 = eul2rotm([deg2rad(-3.6),0,0]);
rotd1 = rotd1(1:2,1:2);
trajd1 = [35.2;46.1];
wallcolor = 'r';
trajcolor = 'm';
bfd1 = [];
% wc = [wallcloud;zeros(1,size(wallcloud,2))];
wc_U = wallcloud;
wc_update=[];
traj_data=[];
td = [];


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
    %% Original initial
    initRot = rotd1;
    initPos = trajd1;
    wc_U = [wc_U wc_update];

    afd1tmp = afd1;
    afd1tmp(1:2,:)=initRot*afd1tmp(1:2,:)+initPos*ones(1,size(afd1tmp(1:2,:),2));
    [TRd1,TTd1]=icp(wc_U,afd1tmp,iter,'Matching','kDtree','WorstRejection',wr);
    rotd1 = TRd1(1:2,1:2)*initRot;
    trajd1 = TRd1(1:2,1:2)*initPos+TTd1(1:2,1);
    afd1tmp(1:2,:)=TRd1(1:2,1:2)*afd1tmp(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1tmp(1:2,:),2));
    [TRd1,TTd1,p_indxd1,q_indxd1]=icpMatch(wc_U,afd1tmp,iter,'Matching','kDtree',...
        'WorstRejection',dr,'UnmatchDistance',0.5);
    rotd1 = TRd1(1:2,1:2)*rotd1;
    trajd1 = TRd1(1:2,1:2)*trajd1+TTd1(1:2,1);
    afd1tmp(1:2,:)=TRd1(1:2,1:2)*afd1tmp(1:2,:)+TTd1(1:2,1)*ones(1,size(afd1tmp(1:2,:),2));
    tjd1= trajd1;


    % Prevent perturbation
    if it>1
        trajvec = trajd1-traj0; % Moving vector
        disp('before')
        disp(trajvec)
        [d,v] = cart2pol(trajvec(1),trajvec(2));
        trajdeg = rad2deg(d)+offset;
        % Handle jumping degree near 0 and 360 degree
        if it == 2
            d0 = trajdeg;
        elseif it > 2 & trajdeg-d0>300
            trajdeg = trajdeg-360;
            offset = offset-360;
        elseif it > 2 & trajdeg-d0<-300
            trajdeg = trajdeg+360;
            offset = offset+360;
        end 
        % Find perturbation
        if it > 2 & abs(trajdeg-d0) > 45 & it > 80 
            disp(frame)
            disp(it)
            sprintf('angle: %f %f',trajdeg,d0)

            % rotTrue = eul2rotm(deg2rad([imuang(frame) 0 0]));
            degdiff = imuang(frame)-imuang(frame-1);
            degdiff = eul2rotm(deg2rad([degdiff 0 0]));
            initPos = degdiff(1:2,1:2)*movVec*0.7+initPos;
            trajd1 = initPos;
            trajvec = trajd1-traj0; % Moving vector
            disp('after')
            disp(trajvec)
            trajdeg = rotm2eul([initRot*degdiff(1:2,1:2) [0;0];0 0 1]);
            trajdeg = rad2deg(trajdeg(1))+90;
            disp(trajdeg)
            afd1(1:2,:)=initRot*degdiff(1:2,1:2)*afd1(1:2,:)+initPos*ones(1,size(afd1(1:2,:),2));
        else
            afd1 = afd1tmp;
        end
        movVec = trajvec;
        traj0 = trajd1;
        d0 = trajdeg;
        td = [td;trajdeg];
    else
        traj0 = trajd1;
        offset = 0;
    end

    % Update map
    p = afd1(:,p_indxd1);
    wc_update=[wc_update p];
    
    hold on
    scatter(wc_update(1,:),wc_update(2,:),'filled','MarkerFaceColor','c','SizeData',3);
    hold on
    wall=scatter(afd1(1,:),afd1(2,:),'filled','MarkerFaceColor',wallcolor,'SizeData',3);
    hold on
    
    % Angle of view
    maxDist = max(max(v1_a),max(v2_a));
    oriAng = rotm2eul([rotd1 [0;0];0 0 0]);
    oriAng = oriAng(1);
    maxAng = max(max(d1_a),max(d2_a))+oriAng-pi/18;  % Radian
    minAng = min(min(d1_a),min(d2_a))+oriAng+pi/18;  % Radian
    [maskx,masky]=pol2cart([maxAng minAng],[maxDist maxDist]);
    line1 = line([trajd1(1) trajd1(1)+maskx(1)], [trajd1(2) trajd1(2)+masky(1)]);
    hold on
    line2 = line([trajd1(1) trajd1(1)+maskx(2)], [trajd1(2) trajd1(2)+masky(2)]);
    hold on

    % Trajectory
    scatter(trajd1(1,:),trajd1(2,:),'filled','MarkerFaceColor',trajcolor,'SizeData',4)
    t = scatter(trajpoint(1,it),trajpoint(2,it),'filled','MarkerFaceColor','k','SizeData',4);
    t1 = scatter(tjd1(1),tjd1(2),'filled','MarkerFaceColor','y','SizeData',4);

    % disp(frame)
    % disp(it)

    xlim([0 size(A,2)/10])
    ylim([0 size(A,1)/10])
    axis equal
    % disp(frame)
    drawnow

    r = [rotd1 [0;0];0 0 1];
    r = rotm2eul(r);
    r = rad2deg(r(1));
    rotarr = [rotarr;r];

    

    traj_data = [traj_data trajd1];
    it = it+1;
    preframe = frame;
    bf = af;
    bfd1 = afd1;
    bfd2 = afd2;

    % if frame >510
    %     waitforbuttonpress;
    %     % break
    % end
    delete(wall)
    delete(line1)
    delete(line2)
    delete(t)
    delete(t1)
end
% Save the wall point cloud
% save('wallcloud.mat','wallcloud','trajectory')

figure
plot(1:3:m/16,gpsdir(1:3:end))
hold on
r = rotarr-rotarr(1);
plot(1:5:m/16,r)
hold on
plot(imuang)
% xlim([500 550]);
legend('GPS','ICP (LiDAR)','IMU')
xlabel('Frames')
ylabel('Angle (degrees)')
title('Direction')

figure
gpsdeg = gpsdir(16:15:end)-gpsdir(1:15:end-15);
icpdeg = r(4:3:end)-r(1:3:end-3);
imudeg = imuang(16:15:end)-imuang(1:15:end-15);
plot(1:15:15*length(gpsdeg),gpsdeg,1:15:15*length(imudeg),imudeg,1:15:15*length(icpdeg),icpdeg)
legend('GPS','IMU','ICP')
title('Angular Velocity')
xlabel('Frames')

figure
rt = traj_data(:,2:end)-traj_data(:,1:end-1);
[d,v]=cart2pol(rt(1,:),rt(2,:));
d1 = rad2deg(d);
for i=2:length(d1)
    if d1(i)-d1(i-1)>300
        d1(i:end)=d1(i:end)-360;
    elseif d1(i)-d1(i-1)<-300
        d1(i:end)=d1(i:end)+360;
    end
end
plot(6:5:5*length(d1)+5,d1)
title('ICP Direction (Modified)')
xlabel('Frames')
ylabel('(Degrees)')
xlim([-100 2000])

figure
d2 = d1-65;
plot(1:3:m/16,gpsdir(1:3:end))
hold on
plot(imuang)
hold on
plot(6:5:5*length(d2)+5,d2) 
xlim([-100 2000])
title('Direction (Modified)')
xlabel('Frames')
ylabel('(Degrees)')

figure
d3 = d2(2:end)-d2(1:end-1);
plot(11:5:5*length(d3)+10,d3)
title('Angular velocity -- ICP')
xlabel('Frames')                
ylabel('Degrees') 

figure
g1=interp1(1:3:m/16,gpsdir(1:3:end),1:m/16);
g2=g1(6:5:end)-g1(1:5:end-5);
i2=imuang(6:5:end)-imuang(1:5:end-5);
plot(6:5:5*length(g2)+5,g2)
hold on
plot(6:5:5*length(i2)+5,i2)
title('Angular velocity -- GPS & IMU')
legend('GPS','IMU','Location','southeast')
xlabel('Frames')
ylabel('Degrees')

figure
plot(6:5:5*length(g2)+5,g2)
hold on
plot(6:5:5*length(i2)+5,i2)
hold on
plot(11:5:5*length(d3)+10,d3)
title('Angular velocity -- GPS, IMU & ICP')
legend('GPS','IMU','ICP','Location','southeast')
xlabel('Frames')
ylabel('Degrees')


disp('END')
cd ~/Dropbox/study/Project/sensor_fusion
hold on
