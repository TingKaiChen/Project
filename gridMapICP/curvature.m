clear
clc
figure

% Read in CSV file and seperate the data
csvfilename ='../read CPEV data/CPEV170523/CPEV_Record_2017_05_23_16_22_21.mat';
mapfilename ='./globalmap/wallcloud_20170522132724.mat';
pgfilename  ='./pose_graph/LoopClosure/lc_20170523162617.mat';


load('../read CPEV data/CPEV170523/CPEV_Record_2017_05_23_16_26_17.mat')

% Parameters
th_dist = 2;
th_angle = 2;   % Degree


for frame=1:1:(m/16)
% frame = 1;
    if frame == 1
        waitforbuttonpress
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
    id1_nz = d1_a ~= pi/2;
    id2_nz = d2_a ~= pi/2;
    id3_nz = d3_a ~= pi/2;
    id4_nz = d4_a ~= pi/2;
    d1_a = d1_a(id1_nz);
    d2_a = d2_a(id2_nz);
    d3_a = d3_a(id3_nz);
    d4_a = d4_a(id4_nz);
    v1_a = v1_a(id1_nz);
    v2_a = v2_a(id2_nz);
    v3_a = v3_a(id3_nz);
    v4_a = v4_a(id4_nz);

    % Nearest point of the same angle
    [~,id1_u] = uniquetol(d1_a);
    [~,id2_u] = uniquetol(d2_a);
    [~,id3_u] = uniquetol(d3_a);
    [~,id4_u] = uniquetol(d4_a);
    D1_a = d1_a(1,id1_u);
    D2_a = d2_a(1,id2_u);
    D3_a = d3_a(1,id3_u);
    D4_a = d4_a(1,id4_u);
    V1_a = v1_a(1,id1_u);
    V2_a = v2_a(1,id2_u);
    V3_a = v3_a(1,id3_u);
    V4_a = v4_a(1,id4_u);
    

    [x1_a, y1_a, z1_a] = sph2cart(d1_a,ones(size(d1_a))*(-1.6)*pi/180,v1_a);
    [x2_a, y2_a, z2_a] = sph2cart(d2_a,ones(size(d2_a))*(-0.8)*pi/180,v2_a);
    [x3_a, y3_a, z3_a] = sph2cart(d3_a,ones(size(d3_a))*(0.8)*pi/180,v3_a); 
    [x4_a, y4_a, z4_a] = sph2cart(d4_a,ones(size(d4_a))*(1.6)*pi/180,v4_a); 

    [X1_a, Y1_a, Z1_a] = sph2cart(D1_a,ones(size(D1_a))*(-1.6)*pi/180,V1_a);
    [X2_a, Y2_a, Z2_a] = sph2cart(D2_a,ones(size(D2_a))*(-0.8)*pi/180,V2_a);
    [X3_a, Y3_a, Z3_a] = sph2cart(D3_a,ones(size(D3_a))*(0.8)*pi/180,V3_a); 
    [X4_a, Y4_a, Z4_a] = sph2cart(D4_a,ones(size(D4_a))*(1.6)*pi/180,V4_a); 

    % Excluded edge points
    occluded_ptID = false(1,length(D1_a));
    depthDiff = V1_a(1:end-1)-V1_a(2:end);
    angleDiff = D1_a(1:end-1)-D1_a(2:end);
    % Distance threshold
    id_greater = depthDiff>th_dist;
    occluded_ptID(id_greater) = true;
    % occluded_ptID(id_greater(2:end)) = true;
    % occluded_ptID(id_greater(3:end)) = true;
    id_smaller = depthDiff<-th_dist;
    occluded_ptID([false id_smaller]) = true;
    % occluded_ptID([false false id_smaller(1:end-1)]) = true;
    % occluded_ptID([false false false id_smaller(1:end-2)]) = true;
    % Angle threshold
    id_angle = abs(angleDiff)>deg2rad(th_angle);
    % occluded_ptID(id_angle(3:end)) = false;
    % occluded_ptID(id_angle(2:end)) = false;
    occluded_ptID(id_angle) = false;
    occluded_ptID([false id_angle]) = false;
    % occluded_ptID([false false id_angle(1:end-1)]) = false;
    % occluded_ptID([false false false id_angle(1:end-2)]) = false;

    X1_nooc = X1_a(~occluded_ptID);
    Y1_nooc = Y1_a(~occluded_ptID);
    Z1_nooc = Z1_a(~occluded_ptID);

    % Calculate the curverture and find some edge points
    l1 = [X1_nooc;Y1_nooc;Z1_nooc];
    % diff_l1 = l1(:,5:end-4)*8-l1(:,4:end-5)-l1(:,3:end-6)-l1(:,2:end-7)-l1(:,1:end-8) ...
    %                          -l1(:,6:end-3)-l1(:,7:end-2)-l1(:,8:end-1)-l1(:,9:end);
    diff_l1 = l1(:,4:end-3)*6-l1(:,3:end-4)-l1(:,2:end-5)-l1(:,1:end-6) ...
                             -l1(:,5:end-2)-l1(:,6:end-1)-l1(:,7:end);
    % diff_l1 = l1(:,3:end-2)*4-l1(:,2:end-3)-l1(:,1:end-4) ...
    %                          -l1(:,4:end-1)-l1(:,5:end);
    % diff_l1 = l1(:,2:end-1)*2-l1(:,1:end-2) ...
    %                          -l1(:,3:end); 
    c = sum(diff_l1.^2,1);
    [~,id_sort] = sort(c);
    id_sort = id_sort+3;

    id_choose = [id_sort(1:floor(length(id_sort)/3)) id_sort(ceil(length(id_sort)*2/3):end)];
    scatter3(X1_nooc(id_choose),Y1_nooc(id_choose),Z1_nooc(id_choose),20,colormap(jet(length(id_choose))),'filled');
    hold on
    % line(X1_nooc,Y1_nooc,Z1_nooc)
    % scatter3(X1_nooc(id_sort),Y1_nooc(id_sort),Z1_nooc(id_sort),20,colormap(jet(length(id_sort))),'filled');
    LineMat = zeros(3,length(X1_a)*2);
    LineMat(:,1:2:end) = [X1_a;Y1_a;Z1_a];
    line(LineMat(1,:),LineMat(2,:),LineMat(3,:),'LineWidth',0.1,'Color','r');
    hold off
    view(2)
    xlim([-40,40])
    ylim([0,80])
    drawnow
    disp(['max: ',num2str(max(c))])
    disp(['min: ',num2str(min(c))])
    % waitforbuttonpress;

% break;
end

% hold on

% % scatter3(x1_a,y1_a,z1_a,5,'k','filled');
% scatter3(X1_a(id_sort),Y1_a(id_sort),Z1_a(id_sort),5,linspace(1,5,length(id_sort)),'filled');
% view(2)

disp('END')
