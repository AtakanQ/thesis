function roadLength = calculateRoadLength(roadPoints)
    % Initialize the total length to 0
    roadLength = 0;
    
    % Get the number of points
    numPoints = size(roadPoints, 1);
    
    % Loop through each pair of consecutive points
    for i = 1:(numPoints-1)
        % Calculate the distance between points i and i+1
        dx = roadPoints(i+1, 1) - roadPoints(i, 1);
        dy = roadPoints(i+1, 2) - roadPoints(i, 2);
        distance = sqrt(dx^2 + dy^2);
                % Add the distance to the total road length
        roadLength = roadLength + distance;
    end
end