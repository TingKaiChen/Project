width = 20;
dist = 10;
rot = deg2rad(5);
rot = eul2rotm([rot,0,0]);
mv = -1;
pt = 5;
x1 = linspace(-width/2, width/2, pt);
x2 = x1;
x3 = x1;
x4 = x1;
y1 = ones(1,pt)*dist;
y2 = y1+1;
y3 = y2+1;
y4 = y3+1;

icp_x = [x1 x2 x3 x4];
icp_y = [y1 y2 y3 y4];
% icp_x_mv = icp_x;
% icp_y_mv = icp_y-1;
bf = [icp_x; icp_y; zeros(1,pt*4)];
af = rot*bf+[0;mv;0]*ones(1,pt*4);

[TR, TT, ER] = icp(af, bf, 200, 'Matching', 'kDtree');
pt = TR(1:2,1:2)*bf(1:2,:)+TT(1:2,1)*ones(1,pt*4);

figure(1)
scatter(0,0,100,'x')
hold on
scatter(bf(1,:),bf(2,:),200,'MarkerEdgeColor','b','LineWidth',5)
hold on 
scatter(af(1,:),af(2,:),100,'filled','MarkerFaceColor','r')
hold off

xlim([-15 15])
ylim([-5 dist+5])

saveas(gcf,'figure/rotmv_bfaf_10.jpg')

figure(2)
scatter(0,0,100,'x')
hold on
scatter(pt(1,:),pt(2,:),200,'MarkerEdgeColor','k','LineWidth',5)
hold on 
scatter(af(1,:),af(2,:),100,'filled','MarkerFaceColor','r')
hold off

xlim([-15 15])
ylim([-5 dist+5])

saveas(gcf,'figure/rotmv_icp_10.jpg')