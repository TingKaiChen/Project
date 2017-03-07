dataA = [rand(1,25)*10; rand(1,25)*10];
figure('Position', [100, 50, 1300, 900])
subplot(2,2,1)
P = scatter(dataA(1, :), dataA(2, :), 'filled');
set(P, 'MarkerFaceColor', 'k');
title('dataA');