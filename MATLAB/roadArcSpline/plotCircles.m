function  plotCircles(curvature,centers,picked_x,picked_y)
allPoints = [];
figure;
for k = 2:(length(picked_x)-1)
    if(k == 2) || (k == length(centers)+1 )
    start_coor = [picked_x(k-1) picked_y(k-1)];
    end_coor = [picked_x(k+1) picked_y(k+1)]; 
    else
    start_coor = [picked_x(k) picked_y(k)];
    end_coor = [picked_x(k+1) picked_y(k+1)];

    end
    diff_start = start_coor - centers(k-1,:);
    diff_end = end_coor - centers(k-1,:);

    start_angle = atan2(diff_start(2), diff_start(1));
    end_angle = atan2(diff_end(2), diff_end(1));

    start_angle(start_angle < 0) = start_angle(start_angle < 0) + 2*pi;
    end_angle(end_angle < 0) = end_angle(end_angle < 0) + 2*pi;
    
    radius = abs(1/curvature(k-1));
    temp = arcSegment_v2(centers(k-1,:), rad2deg(start_angle), rad2deg(end_angle), radius);

    hold on
    temp.plotArc();
    allPoints = [allPoints; temp.Coordinates];
    axis equal
end
title("Real Road Approximated with Arc Spline")
xlabel('x coordinate')
ylabel('y coordinate')
axis equal
end

