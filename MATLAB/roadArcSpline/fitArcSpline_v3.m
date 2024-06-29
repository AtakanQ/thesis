function [clothoidArray,wayPoints] = ...
    fitArcSpline_v3(init_pos,init_tan,init_curv,clothoids_GT,plotOn,...
    shiftedCoords1, shiftedCoords2, xyPairs)

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
xyPairs = [];
allTangents = [];
allCurvatures = [];
for i = 1:numel(clothoids_GT)
    xyPairs = [xyPairs; clothoids_GT(i).allX' clothoids_GT(i).allY';];
    allTangents = [allTangents clothoids_GT(i).allTangent];
    allCurvatures = [allCurvatures clothoids_GT(i).allCurvature];
end




case1 = false;
case2 = false;
case3 = false;
case4 = false;
if(theta0 > 0)
    if(k0 < 0) % case 1
        sigma = k0 * k0 / (2 * theta0);
        hcLength = -2 * theta0 / k0;
        hcCorrectionLengths = hcLength;
        case1 = true;
    else % case 2
        sigma = 0.0001;
        % sigma = 0.0025;
        l1 = k0/sigma + sqrt(k0^2/2/(sigma^2) + theta0/sigma);  
        l2 = l1 - k0/sigma; 
        hcLength = l1 + l2;
        hcCorrectionLengths = [l1; l2];
        case2 = true;
    end
else
    if(k0 < 0) % case 3
        sigma = 0.0001;
        % sigma = 0.0025;
        l1 = k0/(-sigma) + sqrt(k0^2/2/(sigma^2) - theta0/sigma);  
        l2 = l1 - k0/(-sigma); 
        hcLength = l1 + l2;
        hcCorrectionLengths = [l1; l2];
        case3 = true;
    else % case 4
        sigma = k0 * k0 / (2 * theta0);
        hcLength = -2 * theta0 / k0;
        hcCorrectionLengths = hcLength;
        case4 = true;
    end
end

posErrorClothoid = [];
lengthSoFar = 0;
numClothoidsPassed = 1;
numHcCorrectionPassed = 1;

% find how many clothoids the trajectory pass
for k = 1:numel(clothoids_GT)
    if(roadData(k).cumulativeLength > hcLength)
        numClothoidsToPass = k;
        break
    end
end


posErrorWaypoints = wayPoints;
posErrorwaypointCounter = wayPointCounter;

numSections = numel(hcCorrectionLengths) + ...
    numClothoidsToPass - 1;

if length(hcCorrectionLengths) == 2


    % use the rates to generate the next waypoint.
    for i = 1:numSections
        itWasClothoid = false;
        itWasHcCorrection = false;
        minLen = Inf;
    
        for j = numClothoidsPassed:numClothoidsToPass
            if( (minLen + lengthSoFar) > roadData(j).cumulativeLength)
                minLen = roadData(j).cumulativeLength - lengthSoFar;
                itWasClothoid = true;
                itWasHcCorrection = false;  
            else
                break
            end
        end
        
        for n = numHcCorrectionPassed:numel(hcCorrectionLengths)
            if( (minLen + lengthSoFar)> sum(hcCorrectionLengths( 1:n )) )
                minLen = sum(hcCorrectionLengths( 1:n )) - lengthSoFar;
                % minLen = hcCorrectionLengths(n);
                itWasClothoid = false;
                itWasHcCorrection = true; 
            else
                break
            end
        end
    
    
        if itWasClothoid
            numClothoidsPassed = numClothoidsPassed + 1;
        elseif itWasHcCorrection
            numHcCorrectionPassed = numHcCorrectionPassed + 1;
        end
    
        lengthSoFar = lengthSoFar + minLen;

        % get the rates from each component
        for j = 1:numClothoidsToPass
            if(lengthSoFar <= roadData(j).cumulativeLength)
                roadCurvRate = roadData(j).curvRate;
                break
            end
        end
        
        for n = 1:numel(hcCorrectionLengths)
            if(lengthSoFar <= sum(hcCorrectionLengths(1:n)))
                if (n == 1 )% first part
                    if case2
                        hcRate = -sigma;
                    elseif case4
                        hcRate = sigma;
                    else
                        hcRate = sigma;
                    end
                elseif (n == 2)% second part
                    if case2
                        hcRate = sigma;
                    elseif case4
                        hcRate = -sigma;
                    else
                        hcRate = -sigma;
                    end
                    
                end
                break
            end
        end

        posErrorWaypoints(posErrorwaypointCounter + i).curv = ...
            posErrorWaypoints(posErrorwaypointCounter + i - 1).curv + ...
            hcRate * minLen + ...
            roadCurvRate * minLen;
        posErrorWaypoints(posErrorwaypointCounter + i).length = minLen;
    
        %CHANGE THE ORDER DEPENDING ON CURVATURE LENGTH MAYBE? TODO
        tempClothoid = clothoid_v2(posErrorWaypoints(posErrorwaypointCounter + i - 1).pos,...
            posErrorWaypoints(posErrorwaypointCounter + i - 1).tan, ...
            posErrorWaypoints(posErrorwaypointCounter + i - 1).curv, ...
            posErrorWaypoints(posErrorwaypointCounter + i).curv, ...
            posErrorWaypoints(posErrorwaypointCounter + i).length, ...
            0.01);
    
        posErrorClothoid = [posErrorClothoid tempClothoid];
    
        posErrorWaypoints(posErrorwaypointCounter + i).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
        posErrorWaypoints(posErrorwaypointCounter + i).tan = tempClothoid.final_tan;
    end
    % intermediateCurv = init_curv ...
    %     + hcRate*hcCorrectionLengths(1) + ...
    %     roadData(1).curvRate * hcCorrectionLengths(1);
    % 
    % posErrorClothoid(1) = clothoid_v2(init_pos,init_tan, init_curv, intermediateCurv,...
    %            hcCorrectionLengths(1),0.01);
    % 
    % intermediateXY = [posErrorClothoid(1).allX(end) posErrorClothoid(1).allY(end)];
    % intermediateTan = posErrorClothoid(1).final_tan;
    % final_curvature = intermediateCurv ...
    %     -hcRate*hcCorrectionLengths(2) + ...
    %     roadData(1).curvRate * hcCorrectionLengths(2);
    % 
    % posErrorClothoid(2) = clothoid_v2(intermediateXY,intermediateTan, intermediateCurv, final_curvature,...
    %            hcCorrectionLengths(2),0.01);

else
        % use the rates to generate the next waypoint.
    for i = 1:numSections
        itWasClothoid = false;
        itWasHcCorrection = false;
        minLen = Inf;
    
        for j = numClothoidsPassed:numClothoidsToPass
            if( (minLen + lengthSoFar) > roadData(j).cumulativeLength)
                minLen = roadData(j).cumulativeLength - lengthSoFar;
                itWasClothoid = true;
                itWasHcCorrection = false;  
            else
                break
            end
        end
        
        for n = numHcCorrectionPassed:numel(hcCorrectionLengths)
            if( (minLen + lengthSoFar)> sum(hcCorrectionLengths( 1:n )) )
                minLen = sum(hcCorrectionLengths( 1:n )) - lengthSoFar;
                % minLen = hcCorrectionLengths(n);
                itWasClothoid = false;
                itWasHcCorrection = true; 
            else
                break
            end
        end
    
    
        if itWasClothoid
            numClothoidsPassed = numClothoidsPassed + 1;
        elseif itWasHcCorrection
            numHcCorrectionPassed = numHcCorrectionPassed + 1;
        end

        lengthSoFar = lengthSoFar + minLen;

        % get the rates from each component
        for j = 1:numClothoidsToPass
            if(lengthSoFar <= roadData(j).cumulativeLength)
                roadCurvRate = roadData(j).curvRate;
                break
            end
        end
        
        for n = 1:numel(hcCorrectionLengths)
            if(lengthSoFar <= sum(hcCorrectionLengths(1:n)))
                if (n == 1 )% first part
                    if case2
                        hcRate = -sigma;
                    elseif case4
                        hcRate = sigma;
                    else
                        hcRate = sigma;
                    end
                elseif (n == 2)% second part
                    if case2
                        hcRate = sigma;
                    elseif case4
                        hcRate = -sigma;
                    else
                        hcRate = -sigma;
                    end
                    
                end
                break
            end
        end

        posErrorWaypoints(posErrorwaypointCounter + i).curv = ...
            posErrorWaypoints(posErrorwaypointCounter + i - 1).curv + ...
            hcRate * minLen + ...
            roadCurvRate * minLen;
        posErrorWaypoints(posErrorwaypointCounter + i).length = minLen;
    
        %CHANGE THE ORDER DEPENDING ON CURVATURE LENGTH MAYBE? TODO
        tempClothoid = clothoid_v2(posErrorWaypoints(posErrorwaypointCounter + i - 1).pos,...
            posErrorWaypoints(posErrorwaypointCounter + i - 1).tan, ...
            posErrorWaypoints(posErrorwaypointCounter + i - 1).curv, ...
            posErrorWaypoints(posErrorwaypointCounter + i).curv, ...
            posErrorWaypoints(posErrorwaypointCounter + i).length, ...
            0.01);
    
        posErrorClothoid = [posErrorClothoid tempClothoid];
    
        posErrorWaypoints(posErrorwaypointCounter + i).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
        posErrorWaypoints(posErrorwaypointCounter + i).tan = tempClothoid.final_tan;
    
    end
    % final_curvature = init_curv + ...
    %     sigma*hcLength + ...
    %     roadData(1).curvRate*hcLength;
    % posErrorClothoid(1) = clothoid_v2(init_pos,init_tan, init_curv, final_curvature,...
    %            hcCorrectionLengths(1),0.01);
end
% Heading curvature correction to compute the position error
figure;
for i = 1:numel(posErrorClothoid)
    posErrorClothoid(i).plotPlain([1 0 0]);
    hold on
end
for i = 1:numClothoidsPassed
    clothoids_GT(i).plotPlain([0 0 1]);
end


initPosError = norm([clothoids_GT(1).allX(1) clothoids_GT(1).allY(1)] - [wayPoints(1).pos]);
ground_truth_xy = xyPairs;
finalPoint = [posErrorClothoid(end).allX(end) posErrorClothoid(end).allY(end)];
[closestPoint, index] = findClosestPointOnLine(...
    posErrorClothoid(end).allX(end),posErrorClothoid(end).allY(end),...
    posErrorClothoid(end).allTangent(end) + pi/2,ground_truth_xy);
finalPosError = norm(finalPoint - closestPoint);


plot([clothoids_GT(1).allX(1) wayPoints(1).pos(1)],[clothoids_GT(1).allY(1) wayPoints(1).pos(2)],"--",'Color',[0 0 0],'LineWidth',1)

plot(   [finalPoint(1) closestPoint(1)],...
    [finalPoint(2) closestPoint(2)],"-",'Color',[0 0 0],'LineWidth',1)
h4 = plot(shiftedCoords1(:,1),shiftedCoords1(:,2),'--','Color',[0 0 0],'DisplayName','Lane Boundary','LineWidth',1);
plot(shiftedCoords2(:,1),shiftedCoords2(:,2),'--','Color',[0 0 0],'LineWidth',1)

xlabel("xEast (m)","FontSize",13)
ylabel("yNorth (m)","FontSize",13)
title("HCC Maneuver Effect on Position Error","FontSize",13)

text_string = sprintf('%0.2f m', finalPosError);
text((finalPoint(1) + closestPoint(1))/2 + 3, ...
    (finalPoint(2) + closestPoint(2))/2, text_string,'VerticalAlignment','bottom',...
    'HorizontalAlignment', 'left', 'FontSize', 10);

text_string = sprintf('%0.2f m', initPosError);
text((clothoids_GT(1).allX(1) + wayPoints(1).pos(1))/2, ...
    (clothoids_GT(1).allY(1) + wayPoints(1).pos(2))/2, text_string,'VerticalAlignment','bottom',...
    'HorizontalAlignment', 'left', 'FontSize', 10);
h1 = plot(NaN,NaN,'Color',[1 0 0]);
h2 = plot(NaN,NaN,'Color',[0 0 1]);
h3 = plot(NaN,NaN,'-','Color',[0 0 0]);
legend([h1 h2 h3 h4],{'Trajectory','Road Centerline','Errors','Lane Boundary'})

%Find closest point
errorComputationPoint = [posErrorClothoid(end).allX(end) posErrorClothoid(end).allY(end)];
errorComputatonTangent = posErrorClothoid(end).final_tan;
errorComputationCurvature = posErrorClothoid(end).final_curv;
[closest_point, idx] = findClosestPointOnLine(errorComputationPoint(1), errorComputationPoint(2),...
    errorComputatonTangent + pi/2, xyPairs);

positionErrorComputed = norm(closest_point - errorComputationPoint)
tanErrorComputed = rad2deg(allTangents(idx) - errorComputatonTangent)
curvErrorComputed = errorComputationCurvature - allCurvatures(idx)

curr_point = errorComputationPoint;
[closest_point, ~] = findClosestPointOnLine(curr_point(1), curr_point(2),...
    tanErrorComputed + pi/2, xyPairs);
to_closest = [(closest_point-curr_point) 0];
curr_heading = [cos(errorComputatonTangent) sin(errorComputatonTangent) 0];
crossVector = cross(to_closest,curr_heading);
isOnRight = sign(crossVector(3));

%positive if roadcenter is on right
positionError = sign(isOnRight)*norm(closest_point - errorComputationPoint);

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


numSections = numel(hcCorrectionLengths) + ...
    numel(biElementaryLengths) + ...
    numClothoidsToPass - 2;
%================================%
clothoidArray = [];
wayPointsLength = 0;
numClothoidsPassed = 1;
numBiElementaryPassed = 1;
numHcCorrectionPassed = 1;

%plot
if plotOn
    figure;
    biElemLens = 0;
    for i = 1:5
        biElemLens = [biElemLens sum(biElementaryLengths(1:i))];
    end
    biElemCurv = [0 k_peak 0 0 -k_peak 0];
    plot(biElemLens,biElemCurv,"LineWidth",1.2,"DisplayName","Bi-elementary Curvature")
    hold on
    roadLens = 0;
    roadCurvs = clothoids_GT.init_curv;
    for i = 1:numClothoidsToPass
        roadLens = [roadLens roadData(i).cumulativeLength];
        roadCurvs(i+1) = roadLens(i+1) * roadData(i).curvRate;
    end
    roadCurvs(2) = 0.1;
    ylim([-0.2 0.2])

    plot(roadLens,roadCurvs,"LineWidth",1.2,"DisplayName","Road Curvature")
    hccPlotLens = 0;
    hccCurvatures = k0;
    for i = 1:numel(hcCorrectionLengths)
        hccPlotLens = [hccPlotLens hcCorrectionLengths(i)];
        k1 = hccCurvatures(i) * hcCorrectionLengths(i);
        hccCurvatures(i+1) = k1;
    end
    plot(hccPlotLens,hccCurvatures,"LineWidth",1.2,"DisplayName","HCC Curvature")
    xlim([0 hcLength])
    legend();
    title("Curvature Rates of Each Maneuver","FontSize",13)
    ylabel("Curvature (m^-^1)","FontSize",13)
    xlabel("Arc Length (m)","FontSize",13)
    grid on
    % saveas(gcf,"curvaturesOfManeuvers.png")
end
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
            if (n == 1 )% first part
                if case2
                    hcRate = -sigma;
                elseif case4
                    hcRate = sigma;
                else
                    hcRate = sigma;
                end
            elseif (n == 2)% second part
                if case2
                    hcRate = sigma;
                elseif case4
                    hcRate = -sigma;
                else
                    hcRate = -sigma;
                end
                
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