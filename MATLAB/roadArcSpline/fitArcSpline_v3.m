function [clothoidArray,wayPoints] = ...
    fitArcSpline_v3(init_pos,init_tan,init_curv,clothoids_GT)

wayPoints.pos = init_pos;
wayPoints.tan = init_tan;
wayPoints.curv = init_curv;
wayPoints.length = 0; % length from previous waypoint
wayPointCounter = 1;

% get data from clothoids
roadData(1).cumulativeLength = 0;
for i = 1:numel(clothoids_GT)
    roadData(i).curvRate = clothoids_GT(i).curv_derivative;
    roadData(i).curvLength = clothoids_GT(i).curv_length;
    if i == 1
        roadData(i).cumulativeLength = roadData(i).curvLength;
    else
        roadData(i).cumulativeLength = roadData(i-1).cumulativeLength + roadData(i).curvLength;
    end
end

%compute the errors
theta0 = -(clothoids_GT(1).allTangent(1) - wayPoints(wayPointCounter).tan);
k0 = -(clothoids_GT(1).allCurvature(1) - wayPoints(wayPointCounter).curv);

xyPairs = [clothoids_GT(1).allX' clothoids_GT(1).allY'];
curr_point = wayPoints(1).pos;
[closest_point, ~] = findClosestPointOnLine(curr_point(1), curr_point(2),...
    wayPoints(wayPointCounter).tan + pi/2, xyPairs);
to_closest = [(closest_point-curr_point) 0];
curr_heading = [cos(wayPoints(wayPointCounter).tan) sin(wayPoints(wayPointCounter).tan) 0];
crossVector = cross(to_closest,curr_heading);
isOnRight = sign(crossVector(3));

%positive if roadcenter is on right
positionError = sign(isOnRight)*norm(closest_point - wayPoints(wayPointCounter).pos);


if(theta0 > 0)
    if(k0 < 0) % case 1
        sigma = k0 * k0 / (2 * theta0);
        hcLength = -2 * theta0 / k0;
        hcCorrectionLengths = hcLength;
    else % case 2
        sigma = 0.0025;
        l1 = k0/sigma + sqrt(k0^2/2/(sigma^2) + theta0/sigma);  
        l2 = l1 - k0/sigma; 
        hcLength = l1 + l2;
        hcCorrectionLengths = [l1; l2];
    end
else
    if(k0 < 0) % case 3
        sigma = 0.0025;
        l1 = k0/sigma + sqrt(k0^2/2/(sigma^2) + theta0/sigma);  
        l2 = l1 - k0/sigma; 
        hcLength = l1 + l2;
        hcCorrectionLengths = [l1; l2];
    else % case 4
        sigma = k0 * k0 / (2 * theta0);
        hcLength = -2 * theta0 / k0;
        hcCorrectionLengths = hcLength;
    end
end

%Compute bi-elementary
positionCompensationLength = hcLength;
deltaY = positionError; 
lambda = 0.5; 
gamma = 0.5;
[alpha,k_peak] = computeAlphak1...
(positionCompensationLength,lambda,gamma,-deltaY,0);
baseBiElementaryRate = k_peak / (hcLength / 8);
biElementaryLengths(1) = 1 * hcLength / 8;
biElementaryLengths(2) = 1 * hcLength / 8;
biElementaryLengths(3) = 4 * hcLength / 8;
biElementaryLengths(4) = 1 * hcLength / 8;
biElementaryLengths(5) = 1 * hcLength / 8;

% find how many clothoids the trajectory pass
for k = 1:numel(clothoids_GT)
    if(roadData(k).cumulativeLength > hcLength)
        numClothoidsToPass = k;
        break
    end
end
numSections = numel(hcCorrectionLengths) + ...
    numel(biElementaryLengths) + ...
    numClothoidsToPass - 2;
%================================%
clothoidArray = [];
wayPointsLength = 0;
numClothoidsPassed = 1;
numBiElementaryPassed = 1;
numHcCorrectionPassed = 1;

for i = 1:numSections
    % find the minimum length to determine curvature rates.
    minLen = Inf;
    itWasClothoid = false;
    itWasBiElementary = false;
    itWasHcCorrection = false;

    for j = numClothoidsPassed:numClothoidsToPass
        if( (minLen + wayPointsLength) > roadData(j).cumulativeLength)
            minLen = roadData(j).cumulativeLength - wayPointsLength;
            itWasClothoid = true;
            itWasBiElementary = false;
            itWasHcCorrection = false;  
        else
            break
        end
    end
    
    for k = numBiElementaryPassed:numel(biElementaryLengths)
        if( (minLen  + wayPointsLength) > sum(biElementaryLengths( 1:k )))
            minLen = sum(biElementaryLengths( 1:k )) - wayPointsLength;
            % minLen = biElementaryLengths(k);
            itWasClothoid = false;
            itWasBiElementary = true;
            itWasHcCorrection = false;  
        else
            break
        end
    end
    
    for n = numHcCorrectionPassed:numel(hcCorrectionLengths)
        if( (minLen + wayPointsLength)> sum(hcCorrectionLengths( 1:n )) )
            minLen = sum(hcCorrectionLengths( 1:n )) - wayPointsLength;
            % minLen = hcCorrectionLengths(n);
            itWasClothoid = false;
            itWasBiElementary = false;
            itWasHcCorrection = true; 
        else
            break
        end
    end
    
    if itWasClothoid
        numClothoidsPassed = numClothoidsPassed + 1;
    elseif itWasBiElementary
        numBiElementaryPassed = numBiElementaryPassed + 1;
    elseif itWasHcCorrection
        numHcCorrectionPassed = numHcCorrectionPassed + 1;
    end

    wayPointsLength = wayPointsLength + minLen;

    % get the rates from each component
    for j = 1:numClothoidsToPass
        if(wayPointsLength <= roadData(j).cumulativeLength)
            roadCurvRate = roadData(j).curvRate;
            break
        end
    end
    
    for k = 1:numel(biElementaryLengths)
        if(wayPointsLength <= sum(biElementaryLengths(1:k)) )
            if k == 1
                biElementaryRate = baseBiElementaryRate;
            elseif k == 2
                biElementaryRate = -baseBiElementaryRate;
            elseif k == 3
                biElementaryRate = 0;
            elseif k == 4
                biElementaryRate = -baseBiElementaryRate;
            elseif k == 5
                biElementaryRate = baseBiElementaryRate;
            end

            break
        end
    end
    
    for n = 1:numel(hcCorrectionLengths)
        if(wayPointsLength <= sum(hcCorrectionLengths(1:n)))
            if n == 1 % first part
                hcRate = -sigma;
            elseif n == 2 % second part
                hcRate = sigma;
            end
            break
        end
    end
    
    % use the rates to generate the next waypoint.
    wayPoints(wayPointCounter + i).curv = ...
        wayPoints(wayPointCounter + i - 1).curv + ...
        hcRate * minLen + ...
        biElementaryRate * minLen + ...
        roadCurvRate * minLen;
    wayPoints(wayPointCounter + i).length = minLen;

    %CHANGE THE ORDER DEPENDING ON CURVATURE LENGTH MAYBE? TODO
    tempClothoid = clothoid_v2(wayPoints(wayPointCounter + i - 1).pos,...
        wayPoints(wayPointCounter + i - 1).tan, ...
        wayPoints(wayPointCounter + i - 1).curv, ...
        wayPoints(wayPointCounter + i).curv, ...
        wayPoints(wayPointCounter + i).length, ...
        0.01);

    clothoidArray = [clothoidArray tempClothoid];

    wayPoints(wayPointCounter + i).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
    wayPoints(wayPointCounter + i).tan = tempClothoid.final_tan;
end

end