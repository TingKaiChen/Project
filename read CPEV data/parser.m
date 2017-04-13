%% parser: turn csv file into several matlab objects
function [] = parser(filename, savename)
	% Read in CSV file and seperate the data
	data = csvread(filename);
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

	save(savename,'degmat','deg1','deg2','deg3',...
		'deg4','val1','val2','val3','val4',...
		'xd1','yd1','xd2','yd2','x','y','m','n')
	clc
	exit