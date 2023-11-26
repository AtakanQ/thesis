function [centers] = findCenters(points,tangents,curvatures)
% positive curvature turns left
% negative turns right
num_points = length(curvatures);
centers = zeros(num_points,2);
% 
% figure;
for i = 1:num_points
    radius = abs(1/curvatures(i));
    tangent = tangents(i);
    
    pt_to_center_angle = tangent + sign(curvatures(i)) * pi/2;
    vector_to_center = radius * ...
        [cos(pt_to_center_angle) sin(pt_to_center_angle)];
    centers(i,:) = points(i,:) + vector_to_center;

    % Slope of the tangent line
    % tangent_slope = tan(tangents(i));
    % % Slope of the perpendicular bisector
    % perpendicular_bisector_slope = -1 / tangent_slope;


    % Coordinates of the turning center
    % centers(i,1) = points(i,1) + radius / sqrt(1 + perpendicular_bisector_slope^2);
    % centers(i,2) = points(i,2) + perpendicular_bisector_slope * (centers(i,1) - points(i,1));
    

    % quiver(points(i,1),points(i,2), vector_to_center(1),vector_to_center(2),'LineWidth',1)
    % hold on
    % plot(centers(i,1),centers(i,2),'*')
    % axis equal
end
end

