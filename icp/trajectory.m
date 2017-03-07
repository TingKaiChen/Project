% Read in CSV file and seperate the data
data = csvread('../read CPEV data/test2.csv');
[m, n] = size(data);
degmat = data(1:2:m, :);
val1 = data(2:16:m, :);
val2 = data(4:16:m, :);
val3 = data(6:16:m, :);
val4 = data(8:16:m, :);

% Set valmat and degmat to proper value
% degmat = degmat./(180^2).*2.*(pi^2);  % Radians
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

x = [x1,x2 x3, x4];
y = [y1, y2, y3, y4];


% Video
% outputV = VideoWriter('v2', 'MPEG-4');
% open(outputV)

figure('Name','Trajectory','Position',[100 100 1400 900])


% writeVideo(outputV,getframe(gcf))
step = 3;
rot = [1 0;0 1];
traj = [0;0];
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
RMS1 = zeros(1, ceil(m/16/step));
RMS2 = zeros(1, ceil(m/16/step));
RMS3 = zeros(1, ceil(m/16/step));
RMS4 = zeros(1, ceil(m/16/step));
it = 1;
for frame=1+step:step:(m/16)
    iter = 20;
    if frame~=m/16
        while x(frame, 1)==0
            frame = frame+1;
        end
    end

    nza = size(find(x(frame, :)),2);
    nzb = size(find(x(preframe,:)),2);
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
    af1 = [x1(frame,1:nza1);y1(frame,1:nza1);zeros(1,nza1)];
    bf1 = [x1(preframe,1:nzb1);y1(preframe,1:nzb1);zeros(1,nzb1)];
    af2 = [x2(frame,1:nza2);y2(frame,1:nza2);zeros(1,nza2)];
    bf2 = [x2(preframe,1:nzb2);y2(preframe,1:nzb2);zeros(1,nzb2)];
    af3 = [x3(frame,1:nza3);y3(frame,1:nza3);zeros(1,nza3)];
    bf3 = [x3(preframe,1:nzb3);y3(preframe,1:nzb3);zeros(1,nzb3)];
    af4 = [x4(frame,1:nza4);y4(frame,1:nza4);zeros(1,nza4)];
    bf4 = [x4(preframe,1:nzb4);y4(preframe,1:nzb4);zeros(1,nzb4)];
    [TR, TT, ER] = icp(af, bf, iter, 'Matching', 'kDtree', 'WorstRejection', 0.1);
    [TR1, TT1, ER1] = icp(af1, bf1, iter, 'Matching', 'kDtree', 'WorstRejection', 0.1);
    [TR2, TT2, ER2] = icp(af2, bf2, iter, 'Matching', 'kDtree', 'WorstRejection', 0.1);
    [TR3, TT3, ER3] = icp(af3, bf3, iter, 'Matching', 'kDtree', 'WorstRejection', 0.1);
    [TR4, TT4, ER4] = icp(af4, bf4, iter, 'Matching', 'kDtree', 'WorstRejection', 0.1);
    RMS(it) = ER(iter+1);
    RMS1(it) = ER1(iter+1);
    RMS2(it) = ER2(iter+1);
    RMS3(it) = ER3(iter+1);
    RMS4(it) = ER4(iter+1);
    it = it+1;
    preframe = frame;

    % Trajectory
    rot = TR(1:2,1:2)'*rot;
    traj = traj-rot*TT(1:2,1);
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
    xlim([-10 50])
    ylim([-20 20]) 

%     text(0.3, 0.2, num2str(ER(iter+1), '%.1f'), 'FontSize', 18);
%         text(0.3, 0.2, num2str(norm(TT(1:2,1)), '%.3f'), 'FontSize', 18);

    drawnow
%         writeVideo(outputV,getframe(gcf))

    delete(redTraj)
    delete(redTraj1)
    delete(redTraj2)
    delete(redTraj3)
    delete(redTraj4)
end
% close(outputV)

% RMSbar([RMS1;RMS2;RMS3;RMS4;RMS])

histogram(RMS1)
hold on
histogram(RMS2)
hold on
histogram(RMS3)
hold on
histogram(RMS4)
hold off
