%% Case 1: dx
scatterA

dx = 2;
dataF = [dataA(1, :)+dx; dataA(2, :)];

subplot(2,2,2)
P = scatter(dataA(1, :), dataA(2, :), 'filled');
set(P, 'MarkerFaceColor', 'k');
hold on
P1 = scatter(dataF(1, :), dataF(2, :), 'filled');
set(P1, 'MarkerFaceColor', 'r');
title('dataB');
hold off

[TR, TT, ER, t] = icp([dataF;zeros(1,25)], [dataA;zeros(1,25)]);
newA = TR(1:2,1:2)*dataA+TT(1:2, 1)*ones(1,25);
subplot(2,2,3)
newP =  scatter(newA(1, :), newA(2, :));
set(newP, 'MarkerEdgeColor', 'k', 'LineWidth', 3);
hold on
P1 = scatter(dataF(1, :), dataF(2, :), 'filled');
set(P1, 'MarkerFaceColor', 'r');
title('newA+dataB');
hold off

T = subplot(2,2,4);
set(T,'Visible', 'off')
printTRTT(TR, TT)
text(0.1, 0.2, 'RMS: ', 'FontSize', 18)
text(0.3, 0.2, num2str(ER(11), '%.1f'), 'FontSize', 18)

%% Case 2: dy
scatterA

dy = 2;
dataF = [dataA(1, :); dataA(2, :)+dy];

subplot(2,2,2)
P = scatter(dataA(1, :), dataA(2, :), 'filled');
set(P, 'MarkerFaceColor', 'k');
hold on
P1 = scatter(dataF(1, :), dataF(2, :), 'filled');
set(P1, 'MarkerFaceColor', 'r');
title('dataC');
hold off

[TR, TT, ER, t] = icp([dataF;zeros(1,25)], [dataA;zeros(1,25)]);
newA = TR(1:2,1:2)*dataA+TT(1:2, 1)*ones(1,25);
subplot(2,2,3)
newP =  scatter(newA(1, :), newA(2, :));
set(newP, 'MarkerEdgeColor', 'k', 'LineWidth', 3);
hold on
P1 = scatter(dataF(1, :), dataF(2, :), 'filled');
set(P1, 'MarkerFaceColor', 'r');
title('newA+dataC');
hold off

T = subplot(2,2,4);
set(T,'Visible', 'off')
printTRTT(TR, TT)
text(0.1, 0.2, 'RMS: ', 'FontSize', 18)
text(0.3, 0.2, num2str(ER(11), '%.1f'), 'FontSize', 18)

%% Case 3: dx+dy
scatterA

dx = 3;
dy = 2;
dataF = [dataA(1, :)+dx; dataA(2, :)+dy];

subplot(2,2,2)
P = scatter(dataA(1, :), dataA(2, :), 'filled');
set(P, 'MarkerFaceColor', 'k');
hold on
P1 = scatter(dataF(1, :), dataF(2, :), 'filled');
set(P1, 'MarkerFaceColor', 'r');
title('dataD');
hold off

tic
[TR, TT, ER, t] = icp([dataF;zeros(1,25)], [dataA;zeros(1,25)], 20);
toc
sum(t)
newA = TR(1:2,1:2)*dataA+TT(1:2, 1)*ones(1,25);
subplot(2,2,3)
newP =  scatter(newA(1, :), newA(2, :));
set(newP, 'MarkerEdgeColor', 'k', 'LineWidth', 3);
hold on
P1 = scatter(dataF(1, :), dataF(2, :), 'filled');
set(P1, 'MarkerFaceColor', 'r');
title('newA+dataD');
hold off

T = subplot(2,2,4);
set(T,'Visible', 'off')
printTRTT(TR, TT)
text(0.1, 0.2, 'RMS: ', 'FontSize', 18)
text(0.3, 0.2, num2str(ER(21), '%.1f'), 'FontSize', 18)

%% Case 4: Rotation
theta = pi/6;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

scatterA

dataF = R*dataA;

subplot(2,2,2)
P = scatter(dataA(1, :), dataA(2, :), 'filled');
set(P, 'MarkerFaceColor', 'k');
hold on
P1 = scatter(dataF(1, :), dataF(2, :), 'filled');
set(P1, 'MarkerFaceColor', 'r');
title('dataE');
hold off

iter = 100;
[TR, TT, ER, t] = icp([dataF;zeros(1,25)], [dataA;zeros(1,25)], iter);
newA = TR(1:2,1:2)*dataA+TT(1:2, 1)*ones(1,25);
subplot(2,2,3)
newP =  scatter(newA(1, :), newA(2, :));
set(newP, 'MarkerEdgeColor', 'k', 'LineWidth', 3);
hold on
P1 = scatter(dataF(1, :), dataF(2, :), 'filled');
set(P1, 'MarkerFaceColor', 'r');
title('newA+dataE');
hold off

T = subplot(2,2,4);
set(T,'Visible', 'off')
printTRTT(TR, TT)
text(0.1, 0.2, 'RMS: ', 'FontSize', 18)
text(0.3, 0.2, num2str(ER(iter+1), '%.1f'), 'FontSize', 18)

%% Case 5: Rotation+dx+dy
theta = pi/6;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
dx = 3;
dy = 2;

scatterA

dataF = R*dataA+[dx; dy]*ones(1, 25);

subplot(2,2,2)
P = scatter(dataA(1, :), dataA(2, :), 'filled');
set(P, 'MarkerFaceColor', 'k');
hold on
P1 = scatter(dataF(1, :), dataF(2, :), 'filled');
set(P1, 'MarkerFaceColor', 'r');
title('dataF');
hold off

iter = 30;
[TR, TT, ER, t] = icp([dataF;zeros(1,25)], [dataA;zeros(1,25)], iter);
newA = TR(1:2,1:2)*dataA+TT(1:2, 1)*ones(1,25);
subplot(2,2,3)
newP =  scatter(newA(1, :), newA(2, :));
set(newP, 'MarkerEdgeColor', 'k', 'LineWidth', 3);
hold on
P1 = scatter(dataF(1, :), dataF(2, :), 'filled');
set(P1, 'MarkerFaceColor', 'r');
title('newA+dataF');
hold off

T = subplot(2,2,4);
set(T,'Visible', 'off')
printTRTT(TR, TT)
text(0.1, 0.2, 'RMS: ', 'FontSize', 18)
text(0.3, 0.2, num2str(ER(iter+1), '%.1f'), 'FontSize', 18)
%%
a = [1 2 3;4 5 6;0 0 0];
b = [2 3 4;5 6 7;0 0 0];
[TR, TT, ER, t] = icp(b, a, 10000);
figure
subplot(1,2,1);
scatter(a(1,:),a(2,:), 'MarkerEdgeColor', 'k', 'LineWidth', 3);
hold on 
scatter(b(1,:),b(2,:), 'MarkerFaceColor', 'r');
hold off

c = TR(1:2,1:2)*a(1:2,:)+TT(1:2,1)*ones(1,3);
subplot(1,2,2);
scatter(c(1,:),c(2,:), 'MarkerEdgeColor', 'k', 'LineWidth', 3);
hold on
scatter(b(1,:),b(2,:), 'MarkerFaceColor', 'r');
hold off