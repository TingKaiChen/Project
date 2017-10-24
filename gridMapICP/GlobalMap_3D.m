clc
clear
figure(1)
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

figure(2)
hold on
axis equal

% Read in CSV file and seperate the data
csvfilename ='../read CPEV data/CPEV170523/CPEV_Record_2017_05_23_16_22_21.mat';
mapfilename ='./globalmap/wallcloud_20170522132724.mat';
pgfilename  ='./pose_graph/LoopClosure/lc_20170523162617.mat';


load('../read CPEV data/CPEV170523/CPEV_Record_2017_05_23_16_26_17.mat')

step = 1;
wr   = 0.1;
dr   = 3.0;

% Initial pose
% rot    = eul2rotm([deg2rad(-3.6),0,0]);
rot      = eul2rotm([0,0,0]);
% transl   = [35.2;46.1;0];
transl     = [0;0;0];
wallcloud  = [];
wallcolor  = 'b';
trajcolor  = 'k';
trajectory = [];
timetable  = [];
visible    = false;

quat   = rotm2quat(rot);
vertex = [0, transl', quat];    % [id,tx,ty,tz,q1,q2,q3,q4]
edges  = []; % [id1,id2,tx,ty,tz,q1,q2,q3,q4]

% tic
t_all   = tic;
it      = 1;
edge_it = 1;
e_id    = 0;
for frame=1:step:(m/16)
    t_iter = tic;
    iter = 50;
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
    

    [x1_a, y1_a, z1_a] = sph2cart(d1_a,ones(size(d1_a))*(-1.6)*pi/180,v1_a);
    [x2_a, y2_a, z2_a] = sph2cart(d2_a,ones(size(d2_a))*(-0.8)*pi/180,v2_a);
    [x3_a, y3_a, z3_a] = sph2cart(d3_a,ones(size(d3_a))*(0.8)*pi/180,v3_a); 
    [x4_a, y4_a, z4_a] = sph2cart(d4_a,ones(size(d4_a))*(1.6)*pi/180,v4_a); 

    % ICP
    P_src = [x1_a,x2_a,x3_a,x4_a;y1_a,y2_a,y3_a,y4_a;z1_a,z2_a,z3_a,z4_a];
    P_src_w = P_src;

    if frame ~= 1
        initRot   = rot;
        initPos   = transl;
        P_src_tmp = P_src;
        % Initial condition

        wcdist = sum((wallcloud-transl).^2,1).^0.5;
        wctmp  = wallcloud(:,wcdist<=100);
        P_src_tmp = initRot*P_src_tmp+initPos*ones(1,size(P_src_tmp,2));
        P_src     = initRot*P_src+initPos*ones(1,size(P_src,2));
        [TR12,TT12]=icp(wctmp,P_src_tmp,iter,'Matching','kDtree','WorstRejection',wr);
        initRot = TR12*initRot;
        initPos = TR12*initPos+TT12;
        edge_r  = TR12;
        edge_t  = TT12;

        P_src_tmp =TR12*P_src_tmp+TT12*ones(1,size(P_src_tmp,2));
        P_src     =TR12*P_src+TT12*ones(1,size(P_src,2));
        [TR12,TT12,p_indxd1,q_indxd1]=icpMatch(wctmp,P_src_tmp,iter,'Matching','kDtree',...
            'WorstRejection',dr,'UnmatchDistance',0.5);
        initRot = TR12*initRot;
        initPos = TR12*initPos+TT12;
        edge_r  = TR12*edge_r;
        edge_r  = rot'*edge_r*rot;  % Change basis into initial coordinate
        edge_t  = rot'*(initPos-transl);

        P_src  =TR12*P_src+TT12*ones(1,size(P_src,2));
        rot  = initRot;
        transl = initPos;


        % 
        % P_src=rot*P_src+transl;
        % [TR12,TT12,p_idxd1] = icpMatch(wallcloud, P_src, iter, 'Matching', 'kDtree', 'WorstRejection', dr);

        % rot = TR12*rot;
        % transl = TR12*transl+TT12;
        trajectory = [trajectory transl];

        p = P_src(:,p_indxd1);
        % p = TR12*p+TT12;
        % P_src=TR12*P_src+TT12*ones(1,size(P_src,2));
        % scatter3(px,py,'filled','MarkerFaceColor','b','SizeData',3)
        wallcloud = union(wallcloud',round(p'*10)/10,'rows')';
        % scatter3(p(1,:),p(2,:),'filled','MarkerFaceColor',wallcolor,'SizeData',3);
        wc.XData=[wc.XData p(1,:)];
        wc.YData=[wc.YData p(2,:)];
        wc.ZData=[wc.ZData p(3,:)];
        wc.CData=linspace(1,10,length(wc.XData));
        % wct.XData=wctmp(1,:);
        % wct.YData=wctmp(2,:);
        traj.XData=[traj.XData transl(1,:)];
        traj.YData=[traj.YData transl(2,:)];
        traj.ZData=[traj.ZData transl(3,:)];
        hold on

        % cmpR.XData = wctmp(1,:);
        % cmpR.YData = wctmp(2,:);
        % cmpR.ZData = wctmp(3,:);
        % cmpC.XData = P_src(1,:);
        % cmpC.YData = P_src(2,:);
        % cmpC.ZData = P_src(3,:);
    else
        initRot = rot;
        initPos = transl;
        wallcloud = rot*P_src+transl;
        wallcloud = round(wallcloud*10)/10;
        figure(1)
        wc=scatter3(wallcloud(1,:),wallcloud(2,:),wallcloud(3,:),3,linspace(1,10,size(wallcloud,2)),'filled');
        edge_r = rot;
        edge_r  = rot'*edge_r*rot;
        edge_t = transl;

        % figure(2)
        % hold on
        % cmpR = scatter3(wc.XData,wc.YData,wc.ZData,5,'k','filled');
        % cmpC = scatter3([],[],[],5,'b','filled');
        % cmp3 = scatter3([],[],[],5,'r','filled');

        % wcdist = sum((wallcloud-[transl;0]).^2,1).^0.5;
        % wctmp = wallcloud(:,wcdist<=100);
        % wct = scatter3(wctmp(1,:),wctmp(2,:),3,'k');

        % Trajectory
        figure(1)
        traj = scatter3(transl(1,:),transl(2,:),transl(3,:),10,trajcolor,'filled');

        hold on
    end 

    % Construct pose-graph
    v_quat = rotm2quat(rot);
    e_quat = rotm2quat(edge_r);
    % e_quat = rotm2quat(initRot);
    vertex(it+1,:) = [it, transl', v_quat];
    edges(edge_it,:) = [e_id, it, edge_t', e_quat];
    % edges(edge_it,:) = [0, it, initPos', e_quat];
    edge_it = edge_it+1;

    % ICP cycle edge
    % if mod(frame,3) == 1 & frame > 1
    %     r1 = quat2rotm(vertex(it-2,end-3:end));
    %     t1 = vertex(it-2,2:4)';
    %     initRot   = r1;
    %     initPos   = t1;
    %     P_src = P_src_w;
    %     % Initial condition

    %     wcdist = sum((wallcloud-t1).^2,1).^0.5;
    %     wctmp  = wallcloud(:,wcdist<=100);
    %     P_src     = initRot*P_src+initPos*ones(1,size(P_src,2));
    %     [TR12,TT12]=icp(wctmp,P_src,iter,'Matching','kDtree','WorstRejection',wr);
    %     initRot = TR12*initRot;
    %     initPos = TR12*initPos+TT12;
    %     edge_r  = TR12;
    %     edge_t  = TT12;

    %     P_src     =TR12*P_src+TT12*ones(1,size(P_src,2));
    %     [TR12,TT12,p_indxd1,q_indxd1]=icpMatch(wctmp,P_src,iter,'Matching','kDtree',...
    %         'WorstRejection',dr,'UnmatchDistance',0.5);
    %     initRot = TR12*initRot;
    %     initPos = TR12*initPos+TT12;
    %     edge_r  = TR12*edge_r;
    %     edge_r  = r1'*edge_r*r1;  % Change basis into initial coordinate
    %     edge_t  = r1'*(initPos-t1);

    %     % Add a new cycle edge
    %     e_quat = rotm2quat(edge_r);
    %     edges(edge_it,:) = [it-3, it, edge_t', e_quat];
    %     edge_it = edge_it+1;

    %     P_src  =TR12*P_src+TT12*ones(1,size(P_src,2));
    %     % cmp3.XData = P_src(1,:);
    %     % cmp3.YData = P_src(2,:);
    %     % cmp3.ZData = P_src(3,:);
    % end

    if visible
        figure(1)
        % Angle of view
        maxDist = max(max(v1_a),max(v2_a));
        oriAng = rotm2eul(rot);
        oriAng = oriAng(1);
        maxAng = max(max(d1_a),max(d2_a))+oriAng-pi/18;  % Radian
        minAng = min(min(d1_a),min(d2_a))+oriAng+pi/18;  % Radian
        [maskx,masky]=pol2cart([maxAng minAng],[maxDist maxDist]);

        wall=scatter3(P_src(1,:),P_src(2,:),P_src(3,:),3,'r','filled');
        % Angle of view
        line1 = line([transl(1) transl(1)+maskx(1)], [transl(2) transl(2)+masky(1)]);
        line2 = line([transl(1) transl(1)+maskx(2)], [transl(2) transl(2)+masky(2)]);


        % axis equal
        drawnow
        % xlim([transl(1,end)-20 transl(1,end)+20])
        % ylim([transl(2,end)-20 transl(2,end)+20])
    end


    disp(['Iteration: ', num2str(it), '  Frame: ', num2str(frame)])
    % if mod(frame,3) == 1 & frame > 1800
    %     waitforbuttonpress;
    %     % break
    % end

    timetable(it)=toc(t_iter);

    e_id = it;
    it = it+1;
    preframe = frame;
    bf = P_src;


    if visible
        delete(wall)
        delete(line1)
        delete(line2)
    end

end

% Simple loop closure
edges(edge_it,:) = [e_id, 0, [0 0 0], [1 0 0 0]];

% Order of quaternion in g2o
vertex(:,end-3:end) = [vertex(:,end-2:end) vertex(:,end-3)];
edges(:,end-3:end) = [edges(:,end-2:end), edges(:,end-3)];

% % Save the wall point cloud
% save(mapfilename,'wallcloud','trajectory')

% % Save the pose-graph
save(pgfilename,'vertex','edges')


disp('END')
% cd ~/Dropbox/study/Project/gridMapICP
hold on

toc(t_all)
