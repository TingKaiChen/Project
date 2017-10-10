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
csvfilename ='../read CPEV data/CPEV170523/CPEV_Record_2017_05_23_16_22_21.mat';
mapfilename ='./globalmap/wallcloud_20170522132724.mat';
pgfilename  ='./pose_graph/edge_20170522133150.mat';


load('../read CPEV data/CPEV170522/CPEV_Record_2017_05_22_13_31_50.mat')

step = 1;
wr   = 0.1;
dr   = 3.0;

% Initial pose
% rotd1    = eul2rotm([deg2rad(-3.6),0,0]);
rotd1      = eul2rotm([0,0,0]);
% trajd1   = [35.2;46.1;0];
trajd1     = [0;0;0];
wallcloud  = [];
wallcolor  = 'b';
trajcolor  = 'k';
bfd1       = [];
trajectory = [];
timetable  = [];
visible    = false;

quat   = rotm2quat(rotd1);
vertex = [0, trajd1', quat(2:end), quat(1)];    % [id,tx,ty,tz,q1,q2,q3,q4]
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
    T1 = [rotd1 trajd1;0 0 0 1];
    R1 = rotd1;

    if frame ~= 1
        initRot   = rotd1;
        initPos   = trajd1;
        P_src_tmp = P_src;
        % Initial condition

        wcdist = sum((wallcloud-trajd1).^2,1).^0.5;
        wctmp  = wallcloud(:,wcdist<=100);
        P_src_tmp = initRot*P_src_tmp+initPos*ones(1,size(P_src_tmp,2));
        P_src     = initRot*P_src+initPos*ones(1,size(P_src,2));
        [TRd1,TTd1]=icp(wctmp,P_src_tmp,iter,'Matching','kDtree','WorstRejection',wr);
        initRot = TRd1*initRot;
        initPos = TRd1*initPos+TTd1;
        R12  = TRd1;
        edge_t  = TTd1;

        if sum(abs(rotm2eul(R1*R12/initRot))>1e-3*[1 1 1])>0
            disp('-----------1st ICP--------------')
            disp('Rotation error (rad): R1*R12*R2^T')
            rotm2eul(R1*R12*initRot')
            disp('Rotation error (rad): R2^T*R12*R1')
            rotm2eul(initRot'*R12*R1)
            % break
        end

        P_src_tmp =TRd1*P_src_tmp+TTd1*ones(1,size(P_src_tmp,2));
        P_src     =TRd1*P_src+TTd1*ones(1,size(P_src,2));
        [TRd1,TTd1,p_indxd1,q_indxd1]=icpMatch(wctmp,P_src_tmp,iter,'Matching','kDtree',...
            'WorstRejection',dr,'UnmatchDistance',0.5);
        initRot = TRd1*initRot;
        initPos = TRd1*initPos+TTd1;
        R12  = TRd1*R12;
        % edge_t  = TRd1*edge_t+TTd1;
        % edge_t  = edge_t+(R12-eye(3,3))*trajd1;
        % edge_t  = rotd1'*edge_t;
        edge_t  = rotd1'*(initPos-trajd1);

        if sum(abs(rotm2eul(R1*R12/initRot))>1e-3*[1 1 1])>0
            disp('-----------2nd ICP--------------')
            disp('Rotation error (rad): R1*R12*R2^T')
            rotm2eul(R1*R12*initRot')
            disp('Rotation error (rad): R2^T*R12*R1')
            rotm2eul(initRot'*R12*R1)
            % break
        end

        P_src  =TRd1*P_src+TTd1*ones(1,size(P_src,2));
        rotd1  = initRot;
        trajd1 = initPos;

        


        % 
        % P_src=rotd1*P_src+trajd1;
        % [TRd1,TTd1,p_idxd1] = icpMatch(wallcloud, P_src, iter, 'Matching', 'kDtree', 'WorstRejection', dr);

        % rotd1 = TRd1*rotd1;
        % trajd1 = TRd1*trajd1+TTd1;
        trajectory = [trajectory trajd1];

        p = P_src(:,p_indxd1);
        % p = TRd1*p+TTd1;
        % P_src=TRd1*P_src+TTd1*ones(1,size(P_src,2));
        % scatter3(px,py,'filled','MarkerFaceColor','b','SizeData',3)
        wallcloud = union(wallcloud',round(p'*10)/10,'rows')';
        % scatter3(p(1,:),p(2,:),'filled','MarkerFaceColor',wallcolor,'SizeData',3);
        wc.XData=[wc.XData p(1,:)];
        wc.YData=[wc.YData p(2,:)];
        wc.ZData=[wc.ZData p(3,:)];
        wc.CData=linspace(1,10,length(wc.XData));
        % wct.XData=wctmp(1,:);
        % wct.YData=wctmp(2,:);
        traj.XData=[traj.XData trajd1(1,:)];
        traj.YData=[traj.YData trajd1(2,:)];
        traj.ZData=[traj.ZData trajd1(3,:)];
        hold on
    else
        wallcloud = rotd1*P_src+trajd1;
        wallcloud = round(wallcloud*10)/10;
        wc=scatter3(wallcloud(1,:),wallcloud(2,:),wallcloud(3,:),3,linspace(1,10,size(wallcloud,2)),'filled');
        R12 = rotd1;
        edge_t = trajd1;

        % wcdist = sum((wallcloud-[trajd1;0]).^2,1).^0.5;
        % wctmp = wallcloud(:,wcdist<=100);
        % wct = scatter3(wctmp(1,:),wctmp(2,:),3,'k');

        % Trajectory
        traj = scatter3(trajd1(1,:),trajd1(2,:),trajd1(3,:),10,trajcolor,'filled');

        hold on
    end 

    % Construct pose-graph
    v_quat = rotm2quat(rotd1);
    e_quat = rotm2quat(R12);
    vertex(it+1,:) = [it, trajd1', v_quat(2:end), v_quat(1)];
    edges(edge_it,:) = [e_id, it, edge_t', e_quat(2:end), e_quat(1)];

    T2 = [rotd1 trajd1;0 0 0 1];
    T12 = [R12 edge_t;0 0 0 1];
    E = T12^-1*T1^-1*T2;
    % if sum(E(1:3,end)>(1e-5*[1;1;1])) > 0 | sum(abs(rotm2eul(E(1:3,1:3)))>(1e-3*[1 1 1])) > 0
    %     break
    % end
    % R2 = rotd1;
    % rotm2eul(R12'*R1'*R2)


    if visible
        % Angle of view
        maxDist = max(max(v1_a),max(v2_a));
        oriAng = rotm2eul(rotd1);
        oriAng = oriAng(1);
        maxAng = max(max(d1_a),max(d2_a))+oriAng-pi/18;  % Radian
        minAng = min(min(d1_a),min(d2_a))+oriAng+pi/18;  % Radian
        [maskx,masky]=pol2cart([maxAng minAng],[maxDist maxDist]);

        wall=scatter3(P_src(1,:),P_src(2,:),P_src(3,:),3,'r','filled');
        % Angle of view
        line1 = line([trajd1(1) trajd1(1)+maskx(1)], [trajd1(2) trajd1(2)+masky(1)]);
        line2 = line([trajd1(1) trajd1(1)+maskx(2)], [trajd1(2) trajd1(2)+masky(2)]);


        % axis equal
        drawnow
        % xlim([trajd1(1,end)-20 trajd1(1,end)+20])
        % ylim([trajd1(2,end)-20 trajd1(2,end)+20])
    end


    disp(['Iteration: ', num2str(frame)])
    % if frame >100
    %     % waitforbuttonpress;
    %     break
    % end

    timetable(it)=toc(t_iter);

    e_id = it;
    it = it+1;
    edge_it = edge_it+1;
    preframe = frame;
    bf = P_src;
    bfd1 = P_src;


    if visible
        delete(wall)
        delete(line1)
        delete(line2)
    end

end
% % Save the wall point cloud
% save(mapfilename,'wallcloud','trajectory')

% % Save the pose-graph
% save(pgfilename,'vertex','edges')


disp('END')
% cd ~/Dropbox/study/Project/gridMapICP
hold on

toc(t_all)
