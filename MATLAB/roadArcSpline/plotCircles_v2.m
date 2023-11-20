function [arcSegments] = plotCircles_v2(curvature,centers,xEast,yNorth,num_arc_points)

num_arcs = length(curvature);
arcSegments = cell(1,num_arcs-1);
radii = abs(1./curvature);
% figure;
for i=1:(num_arcs-1)
    init_pos =[xEast(i + 1) yNorth(i + 1)];
    next_pos = [xEast(i + 2) yNorth(i + 2)];

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
title('Arc approximated path (single arc for each path)')
ylabel('m')
xlabel('m')

end

