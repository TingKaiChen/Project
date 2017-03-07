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
val1 = val1/100;
val2 = val2/100;
val3 = val3/100;
val4 = val4/100;

[x1, y1] = pol2cart(deg1, val1);
[x2, y2] = pol2cart(deg2, val2);
[x3, y3] = pol2cart(deg3, val3);
[x4, y4] = pol2cart(deg4, val4);

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

% Find the unique-target degrees
isUnique1 = true(1,length(d1));
isUnique2 = true(1,length(d2));
isUnique3 = true(1,length(d3));
isUnique4 = true(1,length(d4));
for i=2:length(d1)
    if d1(i) == d1(i-1)
        isUnique1(i) = false;
        isUnique1(i-1) = false;
    end
end
for i=2:length(d2)
    if d2(i) == d2(i-1)
        isUnique2(i) = false;
        isUnique2(i-1) = false;
    end
end
for i=2:length(d3)
    if d3(i) == d3(i-1)
        isUnique3(i) = false;
        isUnique3(i-1) = false;
    end
end
for i=2:length(d4)
    if d4(i) == d4(i-1)
        isUnique4(i) = false;
        isUnique4(i-1) = false;
    end
end

% Degrees and values of unique-target
d1 = d1(isUnique1);
d2 = d2(isUnique2);
d3 = d3(isUnique3);
d4 = d4(isUnique4);
v1 = v1(isUnique1);
v2 = v2(isUnique2);
v3 = v3(isUnique3);
v4 = v4(isUnique4);

% Intersection of L1/L2 and L3/L4
[~,id1,id2] = intersect(d1,d2);
[~,id3,id4] = intersect(d3,d4);

% Matching and unique-target points in L1/L2 and L3/L4
d1 = d1(id1);
d2 = d2(id2);
d3 = d3(id3);
d4 = d4(id4);
v1 = v1(id1);
v2 = v2(id2);
v3 = v3(id3);
v4 = v4(id4);

[ux1, uy1] = pol2cart(d1, v1);
[ux2, uy2] = pol2cart(d2, v2);
[ux3, uy3] = pol2cart(d3, v3);
[ux4, uy4] = pol2cart(d4, v4);

figure
scatter(x1(frame,:),y1(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(x2(frame,:),y2(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter([ux1,ux2],[uy1,uy2],'filled','MarkerFaceColor','r')
title('L1/L2 LiDAR map')
xlim([-60 60])
ylim([0 120])
saveas(gcf,'figure/L12map_f1800_L.jpg')

figure
scatter(x1(frame,:),y1(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(x2(frame,:),y2(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter([ux1,ux2],[uy1,uy2],'filled','MarkerFaceColor','r')
title('L1/L2 LiDAR map')
xlim([-20 20])
ylim([0 40])
saveas(gcf,'figure/L12map_f1800_s.jpg')

figure
scatter(x3(frame,:),y3(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(x4(frame,:),y4(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter([ux3,ux4],[uy3,uy4],'filled','MarkerFaceColor','r')
title('L3/L4 LiDAR map')
xlim([-60 60])
ylim([0 120])
saveas(gcf,'figure/L34map_f1800_L.jpg')

figure
scatter(x3(frame,:),y3(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter(x4(frame,:),y4(frame,:),'filled','MarkerFaceColor','k')
hold on
scatter([ux3,ux4],[uy3,uy4],'filled','MarkerFaceColor','r')
title('L3/L4 LiDAR map')
xlim([-20 20])
ylim([0 40])
saveas(gcf,'figure/L34map_f1800_s.jpg')

