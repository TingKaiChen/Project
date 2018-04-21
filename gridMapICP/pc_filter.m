function pc_filtered = pc_filter(pc,th_g)
	t_filter = tic;

	azi = pc(3,:);
	ele = pc(5,:);
	r = pc(4,:);
    [x,y,z] = sph2cart(azi,ele,r);

	% Threshold parameters
	th_dist = 1.5;
	th_angle = deg2rad(1);	% radius
	spNum = 20;
	iter = 50;

	% Ground extraction by direct threshold
	ground_id = z<=th_g;
	pc = pc(:,~ground_id);
	azi = pc(3,:);
	ele = pc(5,:);
	r = pc(4,:);
 	[x,y,z] = sph2cart(azi,ele,r);

 %    % Ground extraction by plane fitting
 %    P = [x;y;z];
	% xrange = (P(1,:)>-2.5 & P(1,:)<-1.5) | (P(1,:)>-0.5 & P(1,:)< 0.5) | (P(1,:)> 1.5 & P(1,:)< 2.5);
	% yrange = (P(2,:)>4.5 & P(2,:)<5.5) | (P(2,:)>9 & P(2,:)<11) | (P(2,:)>-9.5 & P(2,:)<-8.5) | (P(2,:)>-13 & P(2,:)<-11);
	% sample = P(:,xrange & yrange)';
	% % Plane fitting by RANSAC
	% [plane,inliers] = RANSAC(sample',spNum,iter,th_g);
	% % Find ground points
	% z_bar = ([P(1:2,:);ones(1,size(P,2))])'*plane;
	% pc = pc(:,abs(z_bar'-P(3,:))>=th_g);
	% azi = pc(3,:);
	% ele = pc(5,:);
	% r = pc(4,:);
 % 	[x,y,z] = sph2cart(azi,ele,r);


	% Excluded edge points
	pc_filtered = [];
	ele_list = uniquetol(ele);
	for i=1:length(ele_list)
		pc_layer = pc(:,ele==ele_list(i));
		azi_layer = pc_layer(3,:);
		ele_layer = pc_layer(5,:);
		r_layer = pc_layer(4,:);
		excluded_ptID = false(1,size(pc_layer,2));

		depthDiff = r_layer-[r_layer(2:end) r_layer(1)];
		angleDiff = azi_layer-[azi_layer(2:end) azi_layer(1)];

		% Distance threshold
		inverse_id = depthDiff<-th_dist;
		excluded_ptID(depthDiff>th_dist) = true;
		excluded_ptID([inverse_id(end) inverse_id(1:end-1)]) = true;
    	% Angle threshold
		abs_id = abs(angleDiff)>th_angle;
		excluded_ptID(abs_id) = false;
		excluded_ptID([abs_id(end) abs_id(1:end-1)]) = false;

		pc_filtered = [pc_filtered pc_layer(:,~excluded_ptID)];
	end

	t_filtered = toc(t_filter);
	disp(['PC filter takes ',num2str(t_filtered)])
end