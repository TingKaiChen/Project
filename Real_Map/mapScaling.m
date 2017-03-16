clc
cd ~/Dropbox/study/Project/Real_Map
map = imread('rotmap.png');
sz = size(map);
sz_scale = [sz(1),sz(1)*143/92];
scale_map = imresize(map, sz_scale);
% figure
% image(map)
figure
image(scale_map)
imwrite(scale_map,'utmMap.png')