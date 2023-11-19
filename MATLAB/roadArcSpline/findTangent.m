function [tangent, angle_rad] = findTangent(pt1,pt2,curvature_center)

to_center = curvature_center - pt1;
to_next = pt2 - pt1;

R_ccw = [0, -1; 1, 0];
R_cw = [0, 1; -1, 0];

cw_or_ccw_vector = cross( [to_center 0] , [to_next 0]);

if( cw_or_ccw_vector(3) > 0 ) %rotate ccw
    tangent = (R_ccw*to_center')./norm(to_center);
    
elseif( cw_or_ccw_vector(3) < 0 ) % rotate cw
    tangent = (R_cw*to_center')./norm(to_center);
end
angle_rad = mod(atan2(tangent(2),tangent(1)),2*pi);
end

