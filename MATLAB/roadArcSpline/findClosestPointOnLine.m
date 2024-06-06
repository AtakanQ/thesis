function [closestPoint, index] = findClosestPointOnLine(x0, y0, angle, xyPairs)
    % Convert angle from degrees to radians if necessary
    theta = deg2rad(angle);

    % Calculate the direction vector
    dirVec = [cos(theta), sin(theta)];
    
    % Initialize variables
    minDist = inf;
    closestPoint = [];
    index = -1;

    % Iterate over the pairs
    for i = 1:size(xyPairs, 1)
        x = xyPairs(i, 1);
        y = xyPairs(i, 2);

        % Find the projection of point (x, y) onto the line
        % The line can be represented parametrically as L(t) = [x0, y0] + t * dirVec
        % Projection of point P(x, y) onto line L can be found by:
        % P_proj = L(t) where t = dot([x, y] - [x0, y0], dirVec) / dot(dirVec, dirVec)
        
        % Vector from line point to the current point
        pointVec = [x - x0, y - y0];
        
        % Projection scalar
        t = dot(pointVec, dirVec) / dot(dirVec, dirVec);
        
        % Projection point
        projPoint = [x0, y0] + t * dirVec;
        
        % Calculate the Euclidean distance from the current point to the projection point
        dist = norm([x, y] - projPoint);
        
        % Update the closest point and index if the distance is smaller
        if dist < minDist
            minDist = dist;
            closestPoint = [x, y];
            index = i;
        end
    end
end
