clc
clear
% Show the digital map
figure
A = imread('../Real_Map/utmMap.png');
% Grayscale
A(:,:,1)=A(:,:,2);
A(:,:,3)=A(:,:,2);
image([0,size(A,2)/10],[0,size(A,1)/10],flip(A,1),'AlphaData',0.5)
truesize
set(gca,'ydir','normal');
axis equal

hold on

% Load in data of wall point cloud and trajectory
load('wallcloud.mat')

% Load in the trajectory of "add-only" strategy
load('./trajectory/traj_grid_101017.mat')

% Grid map
wc_U = round(wallcloud*10)/10;

% Dilation
min_wc = min(min(wc_U));
%% Index should be positive integer
map_index = round((wc_U-min_wc)*10)+1;  
%% Binary matrix
binmap = full(sparse(map_index(1,:),map_index(2,:),1));
%% Dilation
SE = strel('square',5); % structure element=5*5
dilmap = imdilate(binmap,SE);
%% Return to index representation
[r,c]=find(dilmap);
wc_dil = ([r';c']-1)/10+min_wc;
wc_dil = [wc_dil;zeros(1,length(r))];

scatter(wc_dil(1,:),wc_dil(2,:),'filled','MarkerFaceColor','b','SizeData',3)


cd ~/Dropbox/study/Project/gridMapICP
hold on

