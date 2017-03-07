width = 20;
dist = 20;
rot = deg2rad(10);
rot = eul2rotm([rot,0,0]);
mv = -1;
pt = 5;
x1 = linspace(-width/2, width/2, pt);
x2 = x1;
x3 = x1;
x4 = x1;
y1 = ones(1,pt)*dist+rand(1,pt);
y2 = y1+1+rand(1,pt);
y3 = y2+1+rand(1,pt);
y4 = y3+1+rand(1,pt);

icp_x = [x1 x2];
icp_y = [y1 y2];
% icp_x_mv = icp_x;
% icp_y_mv = icp_y-1;
bf = [icp_x; icp_y; zeros(1,pt*2)];
af = rot*bf+[0;mv;0]*ones(1,pt*2);
% af = [icp_x; icp_y-1; zeros(1,pt*2)];

[TR, TT, ER] = icp(af, bf, 200, 'Matching', 'kDtree');
pts = TR(1:2,1:2)*bf(1:2,:)+TT(1:2,1)*ones(1,pt*2);

figure(1)
scatter(0,0,100,'x')
hold on
scatter(bf(1,:),bf(2,:),200,'MarkerEdgeColor','b','LineWidth',5)
hold on 
scatter(af(1,:),af(2,:),100,'filled','MarkerFaceColor','r')
hold off

xlim([-15 15])
ylim([-5 dist+5])

% saveas(gcf,'figure/rotmv_10_bfaf_20.jpg')

figure(2)
scatter(0,0,100,'x')
hold on
scatter(pts(1,:),pts(2,:),200,'MarkerEdgeColor','k','LineWidth',5)
hold on 
scatter(af(1,:),af(2,:),100,'filled','MarkerFaceColor','r')
hold off

xlim([-15 15])
ylim([-5 dist+5])

% saveas(gcf,'figure/rotmv_10_icp_20.jpg')