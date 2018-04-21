function [best_plane,best_inl] = RANSAC(pc,spNum,iter,thDist)
	% pc: 3xn point cloud matrix
	% spNum: Sample number in each iteration
	% iter: Number of iteration
	% thDist: Threshold of inlier calculation

	t = tic;

	% Parameters
	bestInlNum = 0;
	best_plane = [0;0;0];
	best_inl = [];
	% best_error = 0;

	for i = 1:iter
		% Random sample
		idx = randperm(size(pc,2),spNum);
		sample = pc(:,idx)';

		% Fitting plane to random samples by least-square error method (z=ax+by+c)
		P = [sample(:,1:2),ones(size(sample,1),1)];
		plane = (P'*P)\P'*sample(:,3);

		% Find inliers and its number
		z_bar = ([pc(1:2,:);ones(1,size(pc,2))])'*plane;
		inl = pc(:,abs(z_bar'-pc(3,:))<thDist);
		% err = sum((z_bar'-pc(3,:)).^2)

		% % Plane fitting when ax+by+cz=1
		% P = pc(:,idx)';
		% plane = (P'*P)\P'*ones(size(P,1),1);
		% pts_dist = (pc'*plane-ones(size(pc,2),1))/(sum(plane.^2))^0.5;
		% inl = pc(:,abs(pts_dist)<thDist);

		% Update the best plane
		if size(inl,2)>size(best_inl,2)
			best_plane = plane;
			best_inl = inl;
			% best_error = err;
		end
	end

	t = toc(t);
	% disp(['best error: ',num2str(best_error)])
	% disp(['RANSAC takes ',num2str(t)])
end