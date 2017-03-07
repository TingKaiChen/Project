function printTRTT(TR, TT)
    text(0.1,0.8,'Rotate: ', 'FontSize', 18)
    x = linspace(0.3, 0.8, 10);
    y = linspace(0.7, 1, 10);
    text(x(3), y(7), num2str(TR(1,1), '%.3f'), 'FontSize', 18);
    text(x(7), y(7), num2str(TR(1,2), '%.3f'), 'FontSize', 18);
    text(x(3), y(3), num2str(TR(2,1), '%.3f'), 'FontSize', 18);
    text(x(7), y(3), num2str(TR(2,2), '%.3f'), 'FontSize', 18);
    
    text(0.1, 0.5, 'Transition: ', 'FontSize', 18)
    text(0.4, 0.57, num2str(TT(1,1), '%.3f'), 'FontSize', 18);
    text(0.4, 0.47, num2str(TT(2,1), '%.3f'), 'FontSize', 18);
end
