figure
load('../read CPEV data/out.mat');

[x,y,z]=deg2utm(gpsdata(:,2),gpsdata(:,1));

stFrame = 1;
stFrame = 3;	%20170522133150, 20170523162221 start from frame #3
animate = false;

if animate
	gpstraj = scatter(x(stFrame),y(stFrame),5,'filled');
	for i = stFrame+1:length(x)
		gpstraj.XData = [gpstraj.XData x(i)];
		gpstraj.YData = [gpstraj.YData y(i)];
		drawnow
		disp(i)
		axis equal
	end
else
	gpstraj = scatter(x(stFrame:end),y(stFrame:end),5,'filled');
	axis equal
end

% Rotate the graph
rot=eul2rotm([-pi/4 0 0]);
rot=rot(1:2,1:2);
gps=rot*[x y]';

scatter(gps(1,:),gps(2,:),5,'filled');
axis equal;
ylim([1.7245e6 1.72495e6]);
title('Trajectory(GPS)')

%% Calculate the distance between start and end point
% Find first non-zero GPS position
id = find(x~=0 & y~=0,1);
P_start = [x(id);y(id)];
P_end = [x(end);y(end)];
dist_se = sum((P_start-P_end).^2,1).^0.5