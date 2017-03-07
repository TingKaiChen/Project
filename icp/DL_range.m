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


% Set valmat and degmat to proper value
degmat = degmat./(5760).*(pi)+(pi/2);  % Radians
deg1 = degmat(1:8:m/2, :);
deg2 = degmat(2:8:m/2, :);
deg3 = degmat(3:8:m/2, :);
deg4 = degmat(4:8:m/2, :);
% val1 = val1/100;
% val2 = val2/100;
% val3 = val3/100;
% val4 = val4/100;

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

frame = 1800;
d1 = deg1(frame,:);
d2 = deg2(frame,:);
d3 = deg3(frame,:);
d4 = deg4(frame,:);
v1 = val1(frame,:);
v2 = val2(frame,:);
v3 = val3(frame,:);
v4 = val4(frame,:);

% Remove origins
index1 = d1 ~= pi/2;
index2 = d2 ~= pi/2;
index3 = d3 ~= pi/2;
index4 = d4 ~= pi/2;

d1 = d1(index1);
d2 = d2(index2);
d3 = d3(index3);
d4 = d4(index4);
v1 = v1(index1);
v2 = v2(index2);
v3 = v3(index3);
v4 = v4(index4);

degpt1 = [];
degpt2 = [];
degpt3 = [];
degpt4 = [];

% Calculate target number of each degree
deg = d1(1);
pl = 1;
ptNum1 = zeros(1,3);
for i=2:length(d1)
    if deg ~= d1(i)
        ptNum1(pl) = ptNum1(pl)+1;
        degpt1 = [degpt1;deg pl];
        deg = d1(i);
        pl = 1;
    elseif deg == d1(i)
        pl = pl+1;
    end
    if i == length(d1)
        ptNum1(pl) = ptNum1(pl)+1; % the last degree
        degpt1 = [degpt1; deg pl];
    end
end
deg = d2(1);
pl = 1;
ptNum2 = zeros(1,3);
for i=2:length(d2)
    if deg ~= d2(i)
        ptNum2(pl) = ptNum2(pl)+1;
        degpt2 = [degpt2;deg pl];
        deg = d2(i);
        pl = 1;
    elseif deg == d2(i)
        pl = pl+1;
    end
    if i == length(d2)
        ptNum2(pl) = ptNum2(pl)+1; % the last degree
        degpt2 = [degpt2;deg pl];
    end
end
deg = d3(1);
pl = 1;
ptNum3 = zeros(1,3);
for i=2:length(d3)
    if deg ~= d3(i)
        ptNum3(pl) = ptNum3(pl)+1;
        degpt3 = [degpt3;deg pl];
        deg = d3(i);
        pl = 1;
    elseif deg == d3(i)
        pl = pl+1;
    end
    if i == length(d3)
        ptNum3(pl) = ptNum3(pl)+1; % the last degree
        degpt3 = [degpt3;deg pl];
    end
end
deg = d4(1);
pl = 1;
ptNum4 = zeros(1,3);
for i=2:length(d4)
    if deg ~= d4(i)
        ptNum4(pl) = ptNum4(pl)+1;
        degpt4 = [degpt4;deg pl];
        deg = d4(i);
        pl = 1;
    elseif deg == d4(i)
        pl = pl+1;
    end
    if i == length(d4)
        ptNum4(pl) = ptNum4(pl)+1; % the last degree
        degpt4 = [degpt4;deg pl];
    end
end

% Matching in L1L2/L3L4 level
dl12 = union(degpt1(:,1),degpt2(:,1));  % Total degrees of L1+L2
dl34 = union(degpt3(:,1),degpt4(:,1));  % Total degrees of L3+L4
match12 = [dl12 zeros(size(dl12))];
match34 = [dl34 zeros(size(dl34))];
mmpt12 = [];    
mmpt34 = [];    

% Collect mismatching points in L1 and L2 into mmpt12
for i=1:length(dl12)
    in1 = find(degpt1==dl12(i));
    in2 = find(degpt2==dl12(i));
    if isempty(in1)
        match12(i,2)=0;
        mmindex = find(d2==dl12(i));
        mmpt12 = [mmpt12 [d2(mmindex);v2(mmindex)]];
    elseif isempty(in2)
        match12(i,2)=0;
        mmindex = find(d1==dl12(i));
        mmpt12 = [mmpt12 [d1(mmindex);v1(mmindex)]];
    elseif degpt1(in1,2) ~= degpt2(in2,2)
        match12(i,2)=0;
        mmindex = find(d1==dl12(i));
        mmpt12 = [mmpt12 [d1(mmindex);v1(mmindex)]];
        mmindex = find(d2==dl12(i));
        mmpt12 = [mmpt12 [d2(mmindex);v2(mmindex)]];
    else
        match12(i,2)=degpt1(in1,2);
    end
end
% Collect mismatching points in L3 and L4
for i=1:length(dl34)
    in3 = find(degpt3==dl34(i));
    in4 = find(degpt4==dl34(i));
    if isempty(in3)
        match34(i,2)=0;
        mmindex = find(d4==dl34(i));
        mmpt34 = [mmpt34 [d4(mmindex);v4(mmindex)]];
    elseif isempty(in4)
        match34(i,2)=0;
        mmindex = find(d3==dl34(i));
        mmpt34 = [mmpt34 [d3(mmindex);v3(mmindex)]];
    elseif degpt3(in3,2) ~= degpt4(in4,2)
        match34(i,2)=0;
        mmindex = find(d3==dl34(i));
        mmpt34 = [mmpt34 [d3(mmindex);v3(mmindex)]];
        mmindex = find(d4==dl34(i));
        mmpt34 = [mmpt34 [d4(mmindex);v4(mmindex)]];
    else
        match34(i,2)=degpt3(in3,2);
    end
end

[mmx1,mmy1]=pol2cart(mmpt12(1,:),mmpt12(2,:));
[mmx3,mmy3]=pol2cart(mmpt34(1,:),mmpt34(2,:));

figure
scatter(x1(frame,:),y1(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(x2(frame,:),y2(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(mmx1,mmy1,'filled','MarkerFaceColor','r')
title('L1/L2 LiDAR map')
xlim([-6000 6000])
ylim([0 12000])
saveas(gcf,'figure/L12mm_map_f1800_L.jpg')

figure
scatter(x1(frame,:),y1(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(x2(frame,:),y2(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(mmx1,mmy1,'filled','MarkerFaceColor','r')
title('L1/L2 LiDAR map')
xlim([-2000 2000])
ylim([0 4000])
saveas(gcf,'figure/L12mm_map_f1800_s.jpg')

figure
scatter(x3(frame,:),y3(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(x4(frame,:),y4(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(mmx3,mmy3,'filled','MarkerFaceColor','r')
title('L3/L4 LiDAR map')
xlim([-6000 6000])
ylim([0 12000])
saveas(gcf,'figure/L34mm_map_f1800_L.jpg')

figure
scatter(x3(frame,:),y3(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(x4(frame,:),y4(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(mmx3,mmy3,'filled','MarkerFaceColor','r')
title('L3/L4 LiDAR map')
xlim([-2000 2000])
ylim([0 4000])
saveas(gcf,'figure/L34mm_map_f1800_s.jpg')

% figure
% histogram(match12(:,2),3,'BinWidth',0.4)
% title('Matching between L1/L2')
% xlabel('# of matching points (0 is mismatching)')
% ylabel('# of degrees')
% xlim([0 3])
% figure
% histogram(match34(:,2),3,'BinWidth',0.4)
% title('Matching between L3/L4')
% xlabel('# of matching points (0 is mismatching)')
% ylabel('# of degrees')
% xlim([0 3])

% figure
% scatter(x1(frame,:),y1(frame,:),'filled','MarkerFaceColor','k')
% xlim([-2000 2000])
% ylim([0 4000])
% figure
% bar([ptNum1;ptNum2;ptNum3;ptNum4])
% xlabel('Layer')
% ylabel('Number of degree')

% figure
% diff = abs(val1-val2);
% histogram(diff(1,:),'BinWidth',20)
% hold on 
% histogram(diff(1,:),'BinWidth',100)
% hold off
% xlim([0 1000])



