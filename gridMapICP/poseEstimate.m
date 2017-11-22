clc
clear

% Read in CSV file and seperate the data
pcdName  ='./pointcloud/20170523162617_f30s3.mat';
load('../read CPEV data/CPEV170523/CPEV_Record_2017_05_23_16_26_17.mat')

step = 3;
wr   = 0.1;
dr   = 3.0;

% Initial pose
rot      = eul2rotm([0,0,0]);
transl     = [0;0;0];
wallcloud  = [];
wallcolor  = 'b';
trajcolor  = 'k';
visible    = false;
frame = 30;
iter = 50;


%% PC of the target frame
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
% Spherical PC model
[x1_a, y1_a, z1_a] = sph2cart(d1_a,ones(size(d1_a))*(-1.6)*pi/180,v1_a);
[x2_a, y2_a, z2_a] = sph2cart(d2_a,ones(size(d2_a))*(-0.8)*pi/180,v2_a);
[x3_a, y3_a, z3_a] = sph2cart(d3_a,ones(size(d3_a))*(0.8)*pi/180,v3_a); 
[x4_a, y4_a, z4_a] = sph2cart(d4_a,ones(size(d4_a))*(1.6)*pi/180,v4_a); 
P_tar = [x1_a,x2_a,x3_a,x4_a;y1_a,y2_a,y3_a,y4_a;z1_a,z2_a,z3_a,z4_a];

%% PC of the source frame
d1_a = deg1(frame+step,:);
d2_a = deg2(frame+step,:);
d3_a = deg3(frame+step,:);
d4_a = deg4(frame+step,:);
v1_a = val1(frame+step,:);
v2_a = val2(frame+step,:);
v3_a = val3(frame+step,:);
v4_a = val4(frame+step,:);
% Remove origins
d1_a = d1_a(d1_a ~= pi/2);
d2_a = d2_a(d2_a ~= pi/2);
d3_a = d3_a(d3_a ~= pi/2);
d4_a = d4_a(d4_a ~= pi/2);
v1_a = v1_a(d1_a ~= pi/2);
v2_a = v2_a(d2_a ~= pi/2);
v3_a = v3_a(d3_a ~= pi/2);
v4_a = v4_a(d4_a ~= pi/2);
% Spherical PC model
[x1_a, y1_a, z1_a] = sph2cart(d1_a,ones(size(d1_a))*(-1.6)*pi/180,v1_a);
[x2_a, y2_a, z2_a] = sph2cart(d2_a,ones(size(d2_a))*(-0.8)*pi/180,v2_a);
[x3_a, y3_a, z3_a] = sph2cart(d3_a,ones(size(d3_a))*(0.8)*pi/180,v3_a); 
[x4_a, y4_a, z4_a] = sph2cart(d4_a,ones(size(d4_a))*(1.6)*pi/180,v4_a); 
P_src = [x1_a,x2_a,x3_a,x4_a;y1_a,y2_a,y3_a,y4_a;z1_a,z2_a,z3_a,z4_a];
P_src_origin = P_src;


%% ICP
% Initial condition
initRot   = rot;
initPos   = transl;

[TR12,TT12]=icp(P_tar,P_src,iter,'Matching','kDtree','WorstRejection',wr);
initRot = TR12*initRot;
initPos = TR12*initPos+TT12;
P_src     =TR12*P_src+TT12*ones(1,size(P_src,2));

[TR12,TT12]=icpMatch(P_tar,P_src,iter,'Matching','kDtree',...
    'WorstRejection',dr,'UnmatchDistance',0.5);
initRot = TR12*initRot;
initPos = TR12*initPos+TT12;
P_src  =TR12*P_src+TT12*ones(1,size(P_src,2));

rot  = initRot
eulangle = rotm2eul(rot)
transl = initPos'

% Show the scatter before transformed
figure
hold on
scatter3(P_tar(1,:),P_tar(2,:),P_tar(3,:),5,'k','filled')
scatter3(P_src_origin(1,:),P_src_origin(2,:),P_src_origin(3,:),3,'r','filled')
axis equal
% Show the scatter after transformed
figure
hold on
scatter3(P_tar(1,:),P_tar(2,:),P_tar(3,:),5,'k','filled')
scatter3(P_src(1,:),P_src(2,:),P_src(3,:),3,'b','filled')
axis equal
% Show the overall scatter
figure
hold on
scatter3(P_tar(1,:),P_tar(2,:),P_tar(3,:),5,'k','filled')
scatter3(P_src_origin(1,:),P_src_origin(2,:),P_src_origin(3,:),3,'r','filled')
scatter3(P_src(1,:),P_src(2,:),P_src(3,:),3,'b','filled')
axis equal

disp(['Frame: ', num2str(frame), '  and ', num2str(frame+step)])
disp(['PtNum: ', num2str(size(P_tar,2)), '  and ', num2str(size(P_src,2))])

% Save point cloud
P_tar = P_tar';
P_src_origin = P_src_origin';
save(pcdName,'P_tar','P_src_origin')


disp('END')
hold on

