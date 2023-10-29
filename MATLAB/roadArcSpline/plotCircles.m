function  plotCircles(curvature,centers,picked_x,picked_y)
allPoints = [];
figure;
for k = 1:floor((length(centers))/2)
    start_coor = [picked_x(2*k-1) picked_y(2*k-1)];
    end_coor = [picked_x(2*k+1) picked_y(2*k+1)];
    diff_start = start_coor - centers(2*k,:);
    diff_end = end_coor - centers(2*k,:);

    start_angle = atan2(diff_start(2), diff_start(1));
    end_angle = atan2(diff_end(2), diff_end(1));
    start_angle(start_angle < 0) = start_angle(start_angle < 0) + 2*pi;
    end_angle(end_angle < 0) = end_angle(end_angle < 0) + 2*pi;

    temp = arcSegment(centers(2*k,:), rad2deg(start_angle), rad2deg(end_angle),abs(1/curvature(2*k)) );

    hold on
    temp.plotArc();
    allPoints = [allPoints; temp.Coordinates];
end
title("Real Road Approximated with Arc Spline")
xlabel('x coordinate')
ylabel('y coordinate')
axis equal
end

