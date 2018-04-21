clc
clear
load('../read CPEV data/CPEV170913/out.mat')

frame = 80;
step = 10;
iter = 3;
wr   = 0.1;
dr   = 3.0;
ground_th = -0.5;

% Estimation from B-SHOT
R = [ 0.999575 -0.000919056    0.0291443;
	 0.00188836     0.999445   -0.0332486;
	 -0.0290976    0.0332895     0.999022];
T = [-0.119891;3.68074;-0.239061];


%% PC of the target frame
pc = raw(:,raw(1,:)==frame);
filtered_pc = pc_filter(pc,ground_th);
[x,y,z] = sph2cart(filtered_pc(3,:),filtered_pc(5,:),filtered_pc(4,:));
P_tar = [x;y;z];
%% PC of the source frame
pc = raw(:,raw(1,:)==frame+step);
filtered_pc = pc_filter(pc,ground_th);
[x,y,z] = sph2cart(filtered_pc(3,:),filtered_pc(5,:),filtered_pc(4,:));
P_src = [x;y;z];
P_src_origin = P_src;

% Transform
P_src_af  =R*P_src+T*ones(1,size(P_src,2));


% Show the scatter after transformed
figure
hold on
scatter3(P_tar(1,:),P_tar(2,:),P_tar(3,:),5,'k','filled')
scatter3(P_src_af(1,:),P_src_af(2,:),P_src_af(3,:),5,'b','filled')
axis equal

initRot   = R;
initPos   = T;
% initRot      = eul2rotm([0,0,0]);
% initPos     = [0;0;0];

[TR12,TT12]=icp(P_tar,P_src_af,iter,'Matching','kDtree','WorstRejection',wr);
initRot = TR12*initRot;
initPos = TR12*initPos+TT12;
P_src     =TR12*P_src+TT12*ones(1,size(P_src,2));

[TR12,TT12]=icpMatch(P_tar,P_src,iter,'Matching','kDtree',...
    'WorstRejection',dr,'UnmatchDistance',0.5);
initRot = TR12*initRot;
initPos = TR12*initPos+TT12;
P_src  =TR12*P_src+TT12*ones(1,size(P_src,2));

% Show the scatter after transformed
figure
hold on
scatter3(P_tar(1,:),P_tar(2,:),P_tar(3,:),5,'k','filled')
scatter3(P_src(1,:),P_src(2,:),P_src(3,:),5,'r','filled')
axis equal