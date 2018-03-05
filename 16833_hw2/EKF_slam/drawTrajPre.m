function drawTrajPre(x, P)
    %figure(1)
    hold on;
    drawCovEllipse(x(1:2), P(1:2, 1:2), 'm');
    
end