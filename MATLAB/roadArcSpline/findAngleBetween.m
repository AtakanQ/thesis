function angleBetween = findAngleBetween(startingAngle, endingAngle)
    % Ensure angles are within the range [0, 360)
    startingAngle = mod(startingAngle, 360);
    endingAngle = mod(endingAngle, 360);

    % Find the angle between starting and ending angles
    angleBetween = mod(endingAngle - startingAngle, 360);
end