% Read in CSV file and seperate the data
data = csvread('../read CPEV data/CPEV_Record_2016_08_01_10_10_17.csv');
imu = csvread('../read CPEV data/CPEV_Record_2016_08_01_10_10_17_imu.csv');
gps = csvread('../read CPEV data/CPEV_Record_2016_08_01_10_10_17_gps.csv');
[m, n] = size(data);
degmat = data(1:2:m, :);
val1 = data(2:16:m, :);
val2 = data(4:16:m, :);
val3 = data(6:16:m, :);
val4 = data(8:16:m, :);
pitch = cos(deg2rad(imu(:, end-1)));

% Pitch correction
for i = 1:length(imu)
    val1(i, :) = val1(i, :)*pitch(i);
    val2(i, :) = val2(i, :)*pitch(i);
    val3(i, :) = val3(i, :)*pitch(i);
    val4(i, :) = val4(i, :)*pitch(i);
end

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

% for i=1:m/16
%     id1 = find((val1(i,:)<40)==1);
%     id2 = find((val2(i,:)<40)==1);
%     id3 = find((val3(i,:)<40)==1);
%     id4 = find((val4(i,:)<40)==1);
%     len1 = length(id1);
%     len2 = length(id2);
%     len3 = length(id3);
%     len4 = length(id4);
%     d1 = deg1(i,:);
%     d2 = deg2(i,:);
%     d3 = deg3(i,:);
%     d4 = deg4(i,:);
%     deg1(i,:)=[d1(id1) zeros(1,n-len1)];
%     deg2(i,:)=[d2(id2) zeros(1,n-len2)];
%     deg3(i,:)=[d3(id3) zeros(1,n-len3)];
%     deg4(i,:)=[d4(id4) zeros(1,n-len4)];
%     v1 = val1(i,:);
%     v2 = val2(i,:);
%     v3 = val3(i,:);
%     v4 = val4(i,:);
%     val1(i,:)=[v1(id1) zeros(1,n-len1)];
%     val2(i,:)=[v2(id2) zeros(1,n-len2)];
%     val3(i,:)=[v3(id3) zeros(1,n-len3)];
%     val4(i,:)=[v4(id4) zeros(1,n-len4)];
% end

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


% Video
% outputV = VideoWriter('v2', 'MPEG-4');
% open(outputV)

figure('Name','Trajectory','Position',[100 100 1400 900])


% writeVideo(outputV,getframe(gcf))
step = 3;
wr = 0.1;
eul = [];
td = [];
alpha = [];

rot = [1 0;0 1];
traj = [0;0];
rotd1 = [1 0;0 1];
trajd1 = [0;0];
rotd2 = [1 0;0 1];
trajd2 = [0;0];
rot1 = [1 0;0 1];
traj1 = [0;0];
rot2 = [1 0;0 1];
traj2 = [0;0];
rot3 = [1 0;0 1];
traj3 = [0;0];
rot4 = [1 0;0 1];
traj4 = [0;0];
preframe = 1;
RMS = zeros(1, ceil(m/16/step));
RMSd1 = zeros(1, ceil(m/16/step));
RMSd2 = zeros(1, ceil(m/16/step));
RMS1 = zeros(1, ceil(m/16/step));
RMS2 = zeros(1, ceil(m/16/step));
RMS3 = zeros(1, ceil(m/16/step));
RMS4 = zeros(1, ceil(m/16/step));
it = 1;
for frame=1+step:step:(m/16)
    iter = 20;
    if frame~=m/16
        while xd1(frame, 1)==0
            frame = frame+1;
        end
    end

    nza = size(find(x(frame, :)),2);
    nzb = size(find(x(preframe,:)),2);
    nzad1 = size(find(xd1(frame, :)),2);
    nzbd1 = size(find(xd1(preframe,:)),2);
    nzad2 = size(find(xd2(frame, :)),2);
    nzbd2 = size(find(xd2(preframe,:)),2);
    nza1 = size(find(x1(frame, :)),2);
    nzb1 = size(find(x1(preframe,:)),2);
    nza2 = size(find(x2(frame, :)),2);
    nzb2 = size(find(x2(preframe,:)),2);
    nza3 = size(find(x3(frame, :)),2);
    nzb3 = size(find(x3(preframe,:)),2);
    nza4 = size(find(x4(frame, :)),2);
    nzb4 = size(find(x4(preframe,:)),2);
    af = [x(frame,1:nza);y(frame,1:nza);zeros(1,nza)];
    bf = [x(preframe,1:nzb);y(preframe,1:nzb);zeros(1,nzb)];
    afd1 = [xd1(frame,1:nzad1);yd1(frame,1:nzad1);zeros(1,nzad1)];
    bfd1 = [xd1(preframe,1:nzbd1);yd1(preframe,1:nzbd1);zeros(1,nzbd1)];
    afd2 = [xd2(frame,1:nzad2);yd2(frame,1:nzad2);zeros(1,nzad2)];
    bfd2 = [xd2(preframe,1:nzbd2);yd2(preframe,1:nzbd2);zeros(1,nzbd2)];
    af1 = [x1(frame,1:nza1);y1(frame,1:nza1);zeros(1,nza1)];
    bf1 = [x1(preframe,1:nzb1);y1(preframe,1:nzb1);zeros(1,nzb1)];
    af2 = [x2(frame,1:nza2);y2(frame,1:nza2);zeros(1,nza2)];
    bf2 = [x2(preframe,1:nzb2);y2(preframe,1:nzb2);zeros(1,nzb2)];
    af3 = [x3(frame,1:nza3);y3(frame,1:nza3);zeros(1,nza3)];
    bf3 = [x3(preframe,1:nzb3);y3(preframe,1:nzb3);zeros(1,nzb3)];
    af4 = [x4(frame,1:nza4);y4(frame,1:nza4);zeros(1,nza4)];
    bf4 = [x4(preframe,1:nzb4);y4(preframe,1:nzb4);zeros(1,nzb4)];
    [TR, TT, ER] = icp(af, bf, iter, 'Matching', 'kDtree', 'WorstRejection', wr);
    [TRd1, TTd1, ERd1] = icp(afd1, bfd1, iter, 'Matching', 'kDtree', 'WorstRejection', wr);
    [TRd2, TTd2, ERd2] = icp(afd2, bfd2, iter, 'Matching', 'kDtree', 'WorstRejection', wr);
    [TR1, TT1, ER1] = icp(af1, bf1, iter, 'Matching', 'kDtree', 'WorstRejection', wr);
    [TR2, TT2, ER2] = icp(af2, bf2, iter, 'Matching', 'kDtree', 'WorstRejection', wr);
    [TR3, TT3, ER3] = icp(af3, bf3, iter, 'Matching', 'kDtree', 'WorstRejection', wr);
    [TR4, TT4, ER4] = icp(af4, bf4, iter, 'Matching', 'kDtree', 'WorstRejection', wr);
    eul_all = rotm2eul(TR);
    euld1 = rotm2eul(TRd1);
    euld2 = rotm2eul(TRd2);
    eul1 = rotm2eul(TR1);
    eul2 = rotm2eul(TR2);
    eul3 = rotm2eul(TR3);
    eul4 = rotm2eul(TR4);
    eul = [eul; [eul_all(1) euld1(1) euld2(1) eul1(1) eul2(1) eul3(1) eul4(1)]];
    td = [td; [norm(TT) norm(TTd1) norm(TTd2) norm(TT1) norm(TT2) norm(TT3) norm(TT4)]];
%     apa = atan((norm(TT2)*cos(eul2(1))-norm(TT3)*cos(eul3(1)))/(norm(TT2)*sin(eul2(1))-norm(TT3)*sin(eul3(1))));
%     alpha = [alpha apa];
%     RMS(it) = ER(iter+1);
%     RMS1(it) = ER1(iter+1);
%     RMS2(it) = ER2(iter+1);
%     RMS3(it) = ER3(iter+1);
%     RMS4(it) = ER4(iter+1);
    it = it+1;
    preframe = frame;

    % Trajectory
    rot = TR(1:2,1:2)'*rot;
    traj = traj-rot*TT(1:2,1);
    rotd1 = TRd1(1:2,1:2)'*rotd1;
    trajd1 = trajd1-rotd1*TTd1(1:2,1);
    rotd2 = TRd2(1:2,1:2)'*rotd2;
    trajd2 = trajd2-rotd2*TTd2(1:2,1);
    rot1 = TR1(1:2,1:2)'*rot1;
    traj1 = traj1-rot1*TT1(1:2,1);
    rot2 = TR2(1:2,1:2)'*rot2;
    traj2 = traj2-rot2*TT2(1:2,1);
    rot3 = TR3(1:2,1:2)'*rot3;
    traj3 = traj3-rot3*TT3(1:2,1);
    rot4 = TR4(1:2,1:2)'*rot4;
    traj4 = traj4-rot4*TT4(1:2,1);
    scatter(traj(1),traj(2),'filled','MarkerFaceColor','k');
    redTraj = scatter(traj(1),traj(2),'filled','MarkerFaceColor','r');
    hold on
    scatter(trajd1(1),trajd1(2),'filled','MarkerFaceColor','m');
    redTrajd1 = scatter(trajd1(1),trajd1(2),'filled','MarkerFaceColor','r');
    hold on
    scatter(trajd2(1),trajd2(2),'filled','MarkerFaceColor','c');
    redTrajd2 = scatter(trajd2(1),trajd2(2),'filled','MarkerFaceColor','r');
    hold on
    scatter(traj1(1),traj1(2),'filled','MarkerFaceColor','r');
    redTraj1 = scatter(traj1(1),traj1(2),'filled','MarkerFaceColor','k');
    hold on
    scatter(traj2(1),traj2(2),'filled','MarkerFaceColor','y');
    redTraj2 = scatter(traj2(1),traj2(2),'filled','MarkerFaceColor','k');
    hold on
    scatter(traj3(1),traj3(2),'filled','MarkerFaceColor','g');
    redTraj3 = scatter(traj3(1),traj3(2),'filled','MarkerFaceColor','k');
    hold on
    scatter(traj4(1),traj4(2),'filled','MarkerFaceColor','b');
    redTraj4 = scatter(traj4(1),traj4(2),'filled','MarkerFaceColor','k');
    hold on
    axis equal
    xlim([-10 50])
    ylim([-20 20]) 

%     text(0.3, 0.2, num2str(ER(iter+1), '%.1f'), 'FontSize', 18);
%         text(0.3, 0.2, num2str(norm(TT(1:2,1)), '%.3f'), 'FontSize', 18);

    drawnow
%         writeVideo(outputV,getframe(gcf))

    if mod(it, 50) ~= 0
        delete(redTraj)
        delete(redTrajd1)
        delete(redTrajd2)
        delete(redTraj1)
        delete(redTraj2)
        delete(redTraj3)
        delete(redTraj4)
    end
end
% close(outputV)
saveas(gcf,'figure/traj_pc_s3_20160801101017.jpg')

% histogram(RMS1)
% hold on
% histogram(RMS2)
% hold on
% histogram(RMS3)
% hold on
% histogram(RMS4)
% hold off

accumeul = cumsum(eul/pi*180);
accumtd = cumsum(td);

figure('Name','orientation')
plot(accumeul)
saveas(gcf,'figure/ori_pc_s3_20160801101017.jpg')
figure('Name','distance')
plot(accumtd)
saveas(gcf,'figure/dis_pc_s3_20160801101017.jpg')

