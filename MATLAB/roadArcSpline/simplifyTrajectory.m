function waypoints = simplifyTrajectory(x, y, maximumError)
    % simplifyTrajectory Approximates a given trajectory using line segments
    % with a deviation no greater than maximumError.
    %
    % Inputs:
    % x - vector of x coordinates of the trajectory
    % y - vector of y coordinates of the trajectory
    % maximumError - maximum allowable deviation from the original trajectory
    %
    % Output:
    % waypoints - Nx2 matrix of x and y coordinates of the simplified trajectory

    % Check if the input vectors are the same length
    if length(x) ~= length(y)
        error('X and Y must be the same length.');
    end

    % Start the recursive simplification
    waypoints = rdpSimplify(x, y, maximumError);
end

function newPoints = rdpSimplify(x, y, epsilon)
    % Recursive function using the Ramer-Douglas-Peucker algorithm

    % Find the point with the maximum distance
    [maxDist, index] = maxDistance(x, y);

    % If max distance is greater than epsilon, recursively simplify
    if maxDist > epsilon
        % Recursive call
        recPoints1 = rdpSimplify(x(1:index), y(1:index), epsilon);
        recPoints2 = rdpSimplify(x(index:end), y(index:end), epsilon);

        % Concatenate results, removing the duplicate point
        newPoints = [recPoints1; recPoints2(2:end,:)];
    else
        % Use the endpoints
        newPoints = [x(1), y(1); x(end), y(end)];
    end
end

function [maxDist, index] = maxDistance(x, y)
    % Compute the distance of each point to the line defined by the endpoints

    % Line start and end points
    x1 = x(1); y1 = y(1);
    x2 = x(end); y2 = y(end);

    % Line equation coefficients
    A = y2 - y1;
    B = x1 - x2;
    C = A * x1 + B * y1;

    % Distances from line
    distances = abs(A * x + B * y - C) / sqrt(A^2 + B^2);

    % Maximum distance and its index
    [maxDist, index] = max(distances);
end
