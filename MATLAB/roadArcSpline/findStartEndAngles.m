function [start_angles, end_angles] = findStartEndAngles(x ,y)
    
    number_of_points = length(x);
    start_angles = zeros(number_of_points-2,1);
    end_angles = zeros(number_of_points-2,1);

    diffx = diff(x);
    diffy = diff(y);
    
    angles = atan2(diffy,diffx);
    angles(angles < 0) = angles(angles < 0) + 2*pi;
    start_angles = angles(1:end-2);
    end_angles = angles(3:end);
end

