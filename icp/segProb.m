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

x = x1;
y = y1;

% Video
% outputV = VideoWriter('v2', 'MPEG-4');
% open(outputV)

figure('Name','LiDAR ICP','Position',[100 100 1400 900])
subplot(2,2,1)
scatter(x(1,:),y(1,:),'filled','MarkerFaceColor','k');
xlim([-20 20])
ylim([0 40])

subplot(2,2,3)
traj = [0;0];
scatter(traj(1),traj(2),'filled','MarkerFaceColor','b');
hold on

% writeVideo(outputV,getframe(gcf))
step = 5;
rot = [1 0;0 1];
preframe = 1;
fold = 3;
for frame=1+step:step:(m/16)
    iter = 20;
    sza = size(x(frame,:));
%     szb = size(x(frame-step,:));    
%     if (sza(2)>10) && (szb(2)>10)
    if frame~=m/16
        while x(frame, 1)==0
            frame = frame+1;
        end
    end
    subplot(2,2,1)
    scatter(x(frame, :),y(frame,:),'filled','MarkerFaceColor','r');
    hold on
    scatter(x(preframe, :),y(preframe,:),'filled','MarkerFaceColor','k');
    hold off
    xlim([-20 20])
    ylim([0 40])

    check = false;
    subplot(2,2,2)
    nz = sum(x(frame,:)~=0);
    f = floor(nz/fold);
    af = [x(frame,1:f);y(frame,1:f);zeros(1,f)];
    bf = [x(preframe,1:f);y(preframe,1:f);zeros(1,f)];
    [TR1, TT1, ER1, t1] = icp(af, bf, iter, 'Matching', 'kDtree');
    af = [x(frame,f+1:2*f);y(frame,f+1:2*f);zeros(1,f)];
    bf = [x(preframe,f+1:2*f);y(preframe,f+1:2*f);zeros(1,f)];
    [TR2, TT2, ER2, t2] = icp(af, bf, iter, 'Matching', 'kDtree');
    af = [x(frame,2*f+1:nz);y(frame,2*f+1:nz);zeros(1,nz-2*f)];
    bf = [x(preframe,2*f+1:nz);y(preframe,2*f+1:nz);zeros(1,nz-2*f)];
    [TR3, TT3, ER3, t3] = icp(af, bf, iter, 'Matching', 'kDtree');
    [ER, index] = min([ER1(iter+1),ER2(iter+1),ER3(iter+1)]);
    switch index
        case 1
            TR = TR1;
            TT = TT1;
        case 2
            TR = TR2;
            TT = TT2;
        case 3
            TR = TR3;
            TT = TT3;
    end
    
%         while (ER(iter+1)>0.3) && (iter<200)
%             check = true;
%             iter = iter+20;
%             [TR, TT, ER, t] = icp(af, bf, iter, 'Matching', 'kDtree');
% %             TR = TR';
% %             TT = -TT;
%         end

%     if ER(iter+1) > 0.5
%         check = true;
%         [TR, TT, ER, t] = icp(af, bf, 1000, 'Matching', 'kDtree');
%     end
%     if check == true
%         disp(['frame: ', num2str(frame), ' ER: ', num2str(ER(iter+1))]);
%     end
    newXY = TR(1:2,1:2)*[x(frame,:);y(frame,:)]+TT(1:2,1)*ones(1,sza(2));
    scatter(x(frame, :),y(frame,:),'filled','MarkerFaceColor','r');
    hold on
    scatter(newXY(1, :),newXY(2,:),'MarkerEdgeColor','k');
    hold off
    xlim([-20 20])
    ylim([0 40])
    preframe = frame;

    % Trajectory
    subplot(2,2,3)
    if norm(TT(1:2,1)) <= 0.5
        rot = TR(1:2,1:2)'*rot;
        traj = traj-rot*TT(1:2,1);
%             traj = TR(1:2,1:2)'*traj-TT(1:2,1);
        scatter(traj(1),traj(2),'filled','MarkerFaceColor','b');
        redTraj = scatter(traj(1),traj(2),'filled','MarkerFaceColor','r');
        hold on
        xlim([-10 50])
        ylim([-20 20]) 
    end
    
    T = subplot(2,2,4);
    set(T,'Visible', 'off')
    printTRTT(TR, TT)
    text(0.1, 0.2, 'RMS: ', 'FontSize', 18);
    text(0.3, 0.2, num2str(ER, '%.1f'), 'FontSize', 18);
%         text(0.3, 0.2, num2str(norm(TT(1:2,1)), '%.3f'), 'FontSize', 18);

    drawnow
%         writeVideo(outputV,getframe(gcf))

    h = findobj('type', 'text');
    delete(h)
    delete(redTraj)
end
% close(outputV)


