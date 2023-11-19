function plotMVRCcircles(x,y, centers,bezierCurvature,radius_threshold)

hold on
theta = linspace(0, 2*pi, 1000);
for i = 1:length(centers)
    centerX = centers(i,1);
    centerY = centers(i,2);
    radius = 1/bezierCurvature(i);
    if( abs(radius) < radius_threshold)
        centers(i,:)
        curr_point = [x(i+1) y(i+1)];

        crc_x = centerX + radius * cos(theta);
        crc_y = centerY + radius * sin(theta);
        plot(crc_x, crc_y, 'b', 'LineWidth', 0.2);
        plot(curr_point(1), curr_point(2), '*','LineWidth',1,'Color',[0 1 0]);
        plot([x(i) x(i+2)], [y(i) y(i+2)], '*','LineWidth',1,'Color',[0 0 1]);
        axis equal;
        hold on
    end
end
end

