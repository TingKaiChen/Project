clc
cd ~/Dropbox/study/Project/icp/
% Read in CSV file and seperate the data
data = csvread('../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_25_46.csv');
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

figure('Name','Frame Check_All','Position',[100 100 1200 1000]);

step = 5;
wr = 0.1;

rot = [1 0;0 1];
traj = [0;0];
rotd1 = [1 0;0 1];
trajd1 = [0;0];
rotd2 = [1 0;0 1];
trajd2 = [0;0];
rotd1MF = [1 0;0 1];
trajd1MF = [0;0];
rotd2MF = [1 0;0 1];
trajd2MF = [0;0];

preframe = 400;
d1_b = deg1(preframe,:);
d2_b = deg2(preframe,:);
d3_b = deg3(preframe,:);
d4_b = deg4(preframe,:);
v1_b = val1(preframe,:);
v2_b = val2(preframe,:);
v3_b = val3(preframe,:);
v4_b = val4(preframe,:);
% Remove origins
d1_b = d1_b(d1_b ~= pi/2);
d2_b = d2_b(d2_b ~= pi/2);
d3_b = d3_b(d3_b ~= pi/2);
d4_b = d4_b(d4_b ~= pi/2);
v1_b = v1_b(d1_b ~= pi/2);
v2_b = v2_b(d2_b ~= pi/2);
v3_b = v3_b(d3_b ~= pi/2);
v4_b = v4_b(d4_b ~= pi/2);

[x1_b, y1_b] = pol2cart(d1_b, v1_b);
[x2_b, y2_b] = pol2cart(d2_b, v2_b);
[x3_b, y3_b] = pol2cart(d3_b, v3_b);
[x4_b, y4_b] = pol2cart(d4_b, v4_b);

pt12_b = union([x1_b;y1_b]',[x2_b;y2_b]','rows')';
xd1_b = pt12_b(1,:);
yd1_b = pt12_b(2,:);
pt34_b = union([x3_b;y3_b]',[x4_b;y4_b]','rows')';
xd2_b = pt34_b(1,:);
yd2_b = pt34_b(2,:);
pt_b = union(pt12_b',pt34_b','rows')';
x_b = pt_b(1,:);
y_b = pt_b(2,:);

bf = [x_b;y_b;zeros(size(x_b))];
bfd1 = [xd1_b;yd1_b;zeros(size(xd1_b))];
bfd2 = [xd2_b;yd2_b;zeros(size(xd2_b))];

it = 1;
for frame=preframe+step:step:(m/16)
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

    [TR, TT, ER] = icp(af, bf, iter, 'Matching', 'kDtree', 'WorstRejection', wr);
    [TRd1, TTd1, ERd1] = icp(afd1, bfd1, iter, 'Matching', 'kDtree', 'WorstRejection', wr);
    [TRd2, TTd2, ERd2] = icp(afd2, bfd2, iter, 'Matching', 'kDtree', 'WorstRejection', wr);

    % Trajectory
    rot = TR(1:2,1:2)'*rot;
    traj = traj-rot*TT(1:2,1);
    rotd1 = TRd1(1:2,1:2)'*rotd1;
    trajd1 = trajd1-rotd1*TTd1(1:2,1);
    rotd2 = TRd2(1:2,1:2)'*rotd2;
    trajd2 = trajd2-rotd2*TTd2(1:2,1);

    % % Before transform
    % subplot(2,2,1);
    % scatter(bf(1,:),bf(2,:),'filled','MarkerFaceColor','k');
    % hold on
    % scatter(af(1,:),af(2,:),'MarkerEdgeColor','r');
    % hold off
    % title('Before transform');
    % axis equal
    % xlim([-20 20])
    % ylim([0 40])

    % % After transform (plot new point cloud first)
    % subplot(2,2,2);
    % newXY = TR(1:2,1:2)*bf(1:2,:)+TT(1:2,1)*ones(1,size(bf,2));
    % scatter(newXY(1,:),newXY(2,:),'filled','MarkerFaceColor','b');
    % hold on
    % scatter(af(1,:),af(2,:),'filled','MarkerFaceColor','r');
    % hold off 
    % title('After transform');
    % axis equal
    % xlim([-20 20])
    % ylim([0 40])   

    % % After transform (plot next frame point cloud first)
    % subplot(2,2,4);
    % scatter(af(1,:),af(2,:),'filled','MarkerFaceColor','r');
    % hold on
    % scatter(newXY(1,:),newXY(2,:),'filled','MarkerFaceColor','b');
    % hold off 
    % title('After transform');
    % axis equal
    % xlim([-20 20])
    % ylim([0 40])

    % subplot(2,2,3);
    scatter(traj(1),traj(2),'filled','MarkerFaceColor','k');
    redTraj = scatter(traj(1),traj(2),'filled','MarkerFaceColor','r');
    hold on
    scatter(trajd1(1),trajd1(2),'filled','MarkerFaceColor','m');
    redTrajd1 = scatter(trajd1(1),trajd1(2),'filled','MarkerFaceColor','k');
    hold on
    scatter(trajd2(1),trajd2(2),'filled','MarkerFaceColor','c');
    redTrajd2 = scatter(trajd2(1),trajd2(2),'filled','MarkerFaceColor','r');
    hold on
    title('Trajectory');
    axis equal
    xlim([-10 50])
    ylim([-20 20]) 

    filename = strcat(int2str(preframe),'_',int2str(frame));
    it = it+1;
    preframe = frame;
    bf = af;
    bfd1 = afd1;
    bfd2 = afd2;

    drawnow

    location = 'figure/2016_08_01_10_39_37/';
    % saveas(gcf, strcat(location,filename,'.jpg'));

    if mod(it, 50) ~= 0
        delete(redTraj)
        delete(redTrajd1)
        delete(redTrajd2)
    end

end
disp('End')