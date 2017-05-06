%% parser: turn csv file into several matlab objects
function [] = parser(filename, savename)
	% Filename
	lidarName = strcat(filename,'.csv');
	gpsName = strcat(filename,'_gps.csv');
	imuName = strcat(filename,'_imu.csv');
	disp(lidarName)
	disp(gpsName)
	disp(imuName)

	% Read in CSV file and seperate the data
	lidardata = csvread(lidarName);
	[m, n] = size(lidardata);
	degmat = lidardata(1:2:m, :);
	val1 = lidardata(2:16:m, :);
	val2 = lidardata(4:16:m, :);
	val3 = lidardata(6:16:m, :);
	val4 = lidardata(8:16:m, :);

	% Lidar
	%% Set valmat and degmat to proper value
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

	% GPS
	gpsmat = csvread(gpsName);
	gpsdir = gpsmat(:,1);    % Heading direction
	gpsmat = gpsmat(:,[3,4])/100;
	gpsmat = floor(gpsmat)+(gpsmat-floor(gpsmat))*100/60;
	[gps_x,gps_y,~]=deg2utm(gpsmat(:,1),gpsmat(:,2));
	gpsmat = [gps_x,gps_y];
	%% GPS direction correction
	gpsdir = gpsdir-gpsdir(1);	% Reset
	gpsdir = -1*gpsdir;			% Inverse
	for i=2:length(gpsdir)
		if gpsdir(i)-gpsdir(i-1) > 350
			gpsdir(i:end) = gpsdir(i:end)-360;
		elseif gpsdir(i)-gpsdir(i-1) < -350
			gpsdir(i:end) = gpsdir(i:end)+360;
		end
	end

	% IMU
	imumat = csvread(imuName);
	imuw = imumat(:,9);
	imuang = [];
	imutmp = 0;
	for i=1:length(imuw)
		imutmp = imutmp+imuw(i);
		imuang = [imuang;imutmp];
	end
	%% IMU direction correction
	imuang = imuang-imuang(1);	% Reset
	imuang = imuang*(max(gpsdir)-min(gpsdir))/(max(imuang)-min(imuang));


	save(savename,'degmat','deg1','deg2','deg3',...
		'deg4','val1','val2','val3','val4',...
		'xd1','yd1','xd2','yd2','x','y','m','n',...
		'gpsmat','gpsdir',...
		'imuang','imuw')
	clc
	exit