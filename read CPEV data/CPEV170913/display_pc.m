clc
clear
figure(1)
axis equal
hold on

load('out.mat')
v = VideoWriter('100frame_modified.avi');
open(v);

for frame = raw(1,1):raw(1,end)
	PC_interval = raw(:,raw(1,:)==frame);
	frame
	azi = PC_interval(3,:);
	ele = PC_interval(5,:);
	r = PC_interval(4,:);

	[x,y,z] = sph2cart(azi,ele,r);
	if frame == 0
		wc = scatter3(x,y,z,3,'filled');
	else
		wc.XData = x;
		wc.YData = y;
		wc.ZData = z;
	end
	xlim([-100 100])
	ylim([-100 100])

	F = getframe(gcf);
	writeVideo(v,F)

	pause(0.03);
end
close(v);