function [shiftedCoords1, shiftedCoords2] = shiftCoordinates(coordinates, tangents, laneWidth)
    % Ensure the input sizes are consistent
    if size(coordinates, 2) ~= 2
        error('The input coordinates must be an Nx2 array.');
    end
    if size(tangents, 1) ~= size(coordinates, 1)
        error('The size of tangents must be Nx1, where N is the number of coordinate pairs.');
    end
    if ~isscalar(laneWidth)
        error('laneWidth must be a scalar.');
    end
    
    % Number of points
    N = size(coordinates, 1);
    
    % Preallocate arrays for the shifted coordinates
    shiftedCoords1 = zeros(N, 2);
    shiftedCoords2 = zeros(N, 2);
    
    % Calculate the shift amount
    shiftAmount = laneWidth / 2;
    
    % Compute the shifted coordinates
    for i = 1:N
        % Calculate the shift vectors
        shiftVector1 = shiftAmount * [cos(tangents(i) - pi/2), sin(tangents(i) - pi/2)];
        shiftVector2 = shiftAmount * [cos(tangents(i) + pi/2), sin(tangents(i) + pi/2)];
        
        % Apply the shifts to the original coordinates
        shiftedCoords1(i, :) = coordinates(i, :) + shiftVector1;
        shiftedCoords2(i, :) = coordinates(i, :) + shiftVector2;
    end
end
