function [arcSegments] = plotCircles_v2(curvature,centers,xEast,yNorth,num_arc_points,MVRC)
% Point i is connected to point i+1 using the curvature at point i+1.
if MVRC
    num_arcs = length(curvature);
    arcSegments = cell(1,num_arcs);
    radii = abs(1./curvature);
    % figure;
    for i=1:(num_arcs)
        init_pos =[xEast(i) yNorth(i)];
        next_pos = [xEast(i + 1) yNorth(i + 1)];

        init_tan_vect = init_pos - centers(i,:);
        init_tan_v2 = atan2(init_tan_vect(2),init_tan_vect(1));

        final_tan_vect = next_pos - centers(i,:);
        final_tan_v2 = atan2(final_tan_vect(2),final_tan_vect(1));

        arcSegments{i} = arcSegment_v3(centers(i,:), rad2deg(init_tan_v2), ...
            rad2deg( final_tan_v2), radii(i),num_arc_points);
        hold on
        arcSegments{i}.plotArc();
        axis equal
    end
    title('Arc Approximated Path (MVRC Method)')
    ylabel('m')
    xlabel('m')
else%Ground truth is given.
    num_arcs = length(curvature);
    arcSegments = cell(1,num_arcs);
    radii = abs(1./curvature);
    figure;
    for i=1:(num_arcs)
        init_pos =[xEast(i) yNorth(i)];
        next_pos = [xEast(i + 1) yNorth(i + 1)];

        init_tan_vect = init_pos - centers(i,:);
        init_tan_v2 = atan2(init_tan_vect(2),init_tan_vect(1));

        final_tan_vect = next_pos - centers(i,:);
        final_tan_v2 = atan2(final_tan_vect(2),final_tan_vect(1));

        arcSegments{i} = arcSegment_v3(centers(i,:), rad2deg(init_tan_v2), ...
            rad2deg( final_tan_v2), radii(i),num_arc_points);
        hold on
        arcSegments{i}.plotArc();
        axis equal
    end
    title('Arc Approximated Path (Fitting Method)')
    ylabel('m')
    xlabel('m')
end
hold on
plot(xEast,yNorth,'*','Color',[1 0 0])
end

