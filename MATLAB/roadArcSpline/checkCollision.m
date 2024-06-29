function [collisionArray] = checkCollision(bezier,segment)
laneWidth = 3.6;
numCurves = length(bezier.Curves);
collisionArray = false(numCurves,1);

% Check by measuring distance between centerlline and vehicle corners.
if strcmp(segment.type,"line")
    ground_truth_xy = [segment.allX segment.allY];
else
    ground_truth_xy = [segment.allX' segment.allY'];
end

for i = 1:numCurves
    
    leftBound = zeros(length(bezier.Curvatures{i}),2);
    
    positiveCurvatureIndices = bezier.Curvatures{i} >= 0;

    leftBound(positiveCurvatureIndices,:) = bezier.vehicleBoundaries.rearLeft{i}(positiveCurvatureIndices,:);
    leftBound(~positiveCurvatureIndices,:) = bezier.vehicleBoundaries.frontLeft{i}(~positiveCurvatureIndices,:);
    
    %Check left bound
    [~, ~, errors] = ...
        computeSegmentError(leftBound,ground_truth_xy);
    
    if( sum(find(errors>laneWidth/2)) > 0 ) % there is a collision
        collisionArray(i) = true;
        continue
    end

    rightBound = zeros(length(bezier.Curvatures{i}),2);

    rightBound(positiveCurvatureIndices,:) = bezier.vehicleBoundaries.frontRight{i}(positiveCurvatureIndices,:);
    rightBound(~positiveCurvatureIndices,:) = bezier.vehicleBoundaries.rearRight{i}(~positiveCurvatureIndices,:);

    %Check front right
    [~, ~, errors] = ...
        computeSegmentError(rightBound,ground_truth_xy);

    if( sum(find(errors>laneWidth/2)) > 0 ) % there is a collision
        collisionArray(i) = true;
        continue
    end
    
end

end

