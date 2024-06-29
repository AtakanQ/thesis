function [clothoidArray,wayPoints] = ...
    fitArcSpline_v2(init_pos,init_tan,init_curv,clothoidApprox)

wayPoints.pos = init_pos;
wayPoints.tan = init_tan;
wayPoints.curv = init_curv;
wayPoints.length = 0; % length from previous waypoint
wayPointCounter = 1;
fit = false;
clothoidArray = [];
for i = 1:numel(clothoidApprox.arcTangents)
    theta0 = -(clothoidApprox.arcTangents(i) - wayPoints(wayPointCounter).tan);
    k0 = -(clothoidApprox.curvatures(i) - wayPoints(wayPointCounter).curv);
    sigma = k0 * k0 / (2 * theta0);
    
    xyPairs = [clothoidApprox.allX' clothoidApprox.allY'];
    curr_point = wayPoints(wayPointCounter).pos;
    [closest_point, idx] = findClosestPointOnLine(curr_point(1), curr_point(2),...
        wayPoints(wayPointCounter).tan + pi/2, xyPairs);
    to_closest = [(closest_point-curr_point) 0];
    curr_heading = [cos(wayPoints(wayPointCounter).tan) sin(wayPoints(wayPointCounter).tan) 0];
    crossVector = cross(to_closest,curr_heading);
    isOnRight = sign(crossVector(3));
    positionError = sign(isOnRight)*norm(closest_point - wayPoints(wayPointCounter).pos);

    currArcLen = clothoidApprox.arcLenArray(i);

    % Length needed to fix heading and curvature [hcLength]
    

    if(theta0 > 0)
        
        if(k0 < 0) % init curv is negative, single manuever is enough
            hcLength = -2 * theta0 / k0;
    
            % this arc is enough to compensate for heading and curvature
            % error
            positionCompensationLength = hcLength; % it may be equal to currArcLen?
            deltaY = positionError; 
            lambda = 0.5; 
            gamma = 0.5;
            [alpha,k_peak] = computeAlphak1...
            (positionCompensationLength,lambda,gamma,-deltaY,0);
            if(hcLength < currArcLen)% corrected before arc spline changes curvature
                % There will be as many waypoints as bi elementary curve
                % needs. The algorithm would probably never enter here.
    
                for j = 1:5 % 5 additional waypoints are needed
                    currLen = positionCompensationLength/8;
                    if j == 1
                        biElementaryCurvatureEffect = k_peak;
                    elseif j == 2
                        biElementaryCurvatureEffect = -k_peak;
                    elseif j == 3 % 0 curvature part of bi elementary path
                        currLen = positionCompensationLength/2;
                        biElementaryCurvatureEffect = 0;
                    elseif j == 4
                        biElementaryCurvatureEffect = -k_peak;
                    else %last section
                        biElementaryCurvatureEffect = k_peak;
                    end
    
                    wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        sigma * currLen + ...
                        biElementaryCurvatureEffect;
                    wayPoints(wayPointCounter + j).length = currLen;
                    
                    tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                        wayPoints(wayPointCounter + j - 1).tan, ...
                        wayPoints(wayPointCounter + j - 1).curv, ...
                        wayPoints(wayPointCounter + j).curv, ...
                        wayPoints(wayPointCounter + j).length, ...
                        0.01);
                    clothoidArray = [clothoidArray tempClothoid];
                    wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                    wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                end
                wayPointCounter = wayPointCounter + 5;
                break
            else % this arc is not enough, there will be residual error
                lenRatio = currArcLen / hcLength;
                curvatureRatioBiElementary = k_peak / (hcLength/8);
                if     (lenRatio > (7/8) ) % last part of bi-elementary is problematic
                    problematicIndex = 5;
                    lastSectionLength = currArcLen - 7/8*hcLength;
                elseif (lenRatio > (6/8) ) % 1 before last part is problematic
                    problematicIndex = 4;
                    lastSectionLength = currArcLen - 6/8*hcLength;
                elseif (lenRatio > (2/8) ) % 2 before last part is problematic
                    problematicIndex = 3;
                    lastSectionLength = currArcLen - 2/8*hcLength;
                elseif (lenRatio > (1/8) ) % 3 before last part is problematic
                    problematicIndex = 2;
                    lastSectionLength = currArcLen - 1/8*hcLength;
                else
                    problematicIndex = 1;
                    lastSectionLength = currArcLen;
                end
                problematicCurvatureEffect = curvatureRatioBiElementary * lastSectionLength;
    
                for j = 1:problematicIndex
                    if j ~= problematicIndex
                        currLen = positionCompensationLength/8;
                        if j == 1
                            biElementaryCurvatureEffect = k_peak;
                        elseif j == 2
                            biElementaryCurvatureEffect = -k_peak;
                        elseif j == 3 % 0 curvature part of bi elementary path
                            currLen = positionCompensationLength/2;
                            biElementaryCurvatureEffect = 0;
                        elseif j == 4
                            biElementaryCurvatureEffect = -k_peak;
                        else %last section
                            biElementaryCurvatureEffect = k_peak;
                        end
                    else
                        currLen = lastSectionLength;
                        if j == 1
                            biElementaryCurvatureEffect = problematicCurvatureEffect;
                        elseif j == 2
                            biElementaryCurvatureEffect = -problematicCurvatureEffect;
                        elseif j == 3 % 0 curvature part of bi elementary path
                            biElementaryCurvatureEffect = 0;
                        elseif j == 4
                            biElementaryCurvatureEffect = -problematicCurvatureEffect;
                        else %last section
                            biElementaryCurvatureEffect = problematicCurvatureEffect;
                        end
                    end
    
                    wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        sigma * currLen + ...
                        biElementaryCurvatureEffect;
                    wayPoints(wayPointCounter + j).length = currLen;
                    
                    tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                        wayPoints(wayPointCounter + j - 1).tan, ...
                        wayPoints(wayPointCounter + j - 1).curv, ...
                        wayPoints(wayPointCounter + j).curv, ...
                        wayPoints(wayPointCounter + j).length, ...
                        0.01);
                    clothoidArray = [clothoidArray tempClothoid];
                    wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                    wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                end
                wayPointCounter = wayPointCounter + problematicIndex;
            end
        else % single maneuver is not enough
            
            l1 = k0/sigma + sqrt(k0^2/2/(sigma^2) + theta0/sigma);  
            l2 = l1 - k0/sigma; 
            hcLength = l1 + l2;
    
            % this arc is enough to compensate for heading and curvature
            % error
            positionCompensationLength = hcLength; % it may be equal to currArcLen?
            deltaY = positionError; 
            lambda = 0.5; 
            gamma = 0.5;
            [alpha,k_peak] = computeAlphak1...
            (positionCompensationLength,lambda,gamma,-deltaY,0);
            
            sigmaChangingLenRatio = l1 / hcLength;
            if sigmaChangingLenRatio > (7/8)
                sigmaChangingIndex = 5;
                lengthUntilSigmaChange = l1 - (7/8)*hcLength;
            elseif sigmaChangingLenRatio > (6/8)
                sigmaChangingIndex = 4;
                lengthUntilSigmaChange = l1 - (6/8)*hcLength;
            elseif sigmaChangingLenRatio > (2/8)
                sigmaChangingIndex = 3;
                lengthUntilSigmaChange = l1 - (2/8)*hcLength;
            elseif sigmaChangingLenRatio > (1/8)
                sigmaChangingIndex = 2;
                lengthUntilSigmaChange = l1 - (1/8)*hcLength;
            else
                sigmaChangingIndex = 1;
                lengthUntilSigmaChange = l1;
            end
    
            if(hcLength < currArcLen) % corrected before arc spline changes curvature
                % There will be as many waypoints as bi elementary curve
                % needs and one more for the hc correction part.
                for j = 1:5 % 6 additional waypoints are needed
                    
                    currLen = positionCompensationLength/8;
                    if j == 1
                        biElementaryCurvatureEffect = k_peak;
                    elseif j == 2
                        biElementaryCurvatureEffect = -k_peak;
                    elseif j == 3 % 0 curvature part of bi elementary path
                        currLen = positionCompensationLength/2;
                        biElementaryCurvatureEffect = 0;
                    elseif j == 4
                        biElementaryCurvatureEffect = -k_peak;
                    else %last section
                        biElementaryCurvatureEffect = k_peak;
                    end
    
                    if(j == sigmaChangingIndex) % add a waypoint in the middle
                        partial_k_peak = k_peak * lengthUntilSigmaChange / currLen;
    
                        wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        -sigma * lengthUntilSigmaChange + ...
                        partial_k_peak;
                        wayPoints(wayPointCounter + j).length = lengthUntilSigmaChange;
                    
                        tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                            wayPoints(wayPointCounter + j - 1).tan, ...
                            wayPoints(wayPointCounter + j - 1).curv, ...
                            wayPoints(wayPointCounter + j).curv, ...
                            wayPoints(wayPointCounter + j).length, ...
                            0.01);
                        clothoidArray = [clothoidArray tempClothoid];
                        wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                        wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                        
                        wayPointCounter = wayPointCounter + 1;
    
                        % sigma = -sigma; % this changes sign at this point
                        residualLength = currLen - lengthUntilSigmaChange;
                        partial_residual_k_peak = k_peak * (residualLength) / currLen;
    
                        wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        -sigma * residualLength + ...
                        partial_residual_k_peak;
                        wayPoints(wayPointCounter + j).length = residualLength;
                    
                        tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                            wayPoints(wayPointCounter + j - 1).tan, ...
                            wayPoints(wayPointCounter + j - 1).curv, ...
                            wayPoints(wayPointCounter + j).curv, ...
                            wayPoints(wayPointCounter + j).length, ...
                            0.01);
                        clothoidArray = [clothoidArray tempClothoid];
                        wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                        wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                        continue
                    end
    
                    wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        sigma * currLen + ...
                        biElementaryCurvatureEffect;
                    wayPoints(wayPointCounter + j).length = currLen;
                    
                    tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                        wayPoints(wayPointCounter + j - 1).tan, ...
                        wayPoints(wayPointCounter + j - 1).curv, ...
                        wayPoints(wayPointCounter + j).curv, ...
                        wayPoints(wayPointCounter + j).length, ...
                        0.01);
                    clothoidArray = [clothoidArray tempClothoid];
                    wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                    wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                end
                wayPointCounter = wayPointCounter + 5;
                break
            else % this arc is not enough, there will be residual error
                lenRatio = currArcLen / hcLength;
                curvatureRatioBiElementary = k_peak / (hcLength/8);
                if  (lenRatio > (7/8) ) % last part of bi-elementary is problematic
                    problematicIndex = 5;
                    lastSectionLength = currArcLen - 7/8*hcLength;
                elseif (lenRatio > (6/8) ) % 1 before last part is problematic
                    problematicIndex = 4;
                    lastSectionLength = currArcLen - 6/8*hcLength;
                elseif (lenRatio > (2/8) ) % 2 before last part is problematic
                    problematicIndex = 3;
                    lastSectionLength = currArcLen - 2/8*hcLength;
                elseif (lenRatio > (1/8) ) % 3 before last part is problematic
                    problematicIndex = 2;
                    lastSectionLength = currArcLen - 1/8*hcLength;
                else
                    problematicIndex = 1;
                    lastSectionLength = currArcLen;
                end
                problematicCurvatureEffect = curvatureRatioBiElementary * lastSectionLength;
    
                for j = 1:problematicIndex
                    if j ~= problematicIndex
                        currLen = positionCompensationLength/8;
                        if j == 1
                            biElementaryCurvatureEffect = k_peak;
                        elseif j == 2
                            biElementaryCurvatureEffect = -k_peak;
                        elseif j == 3 % 0 curvature part of bi elementary path
                            currLen = positionCompensationLength/2;
                            biElementaryCurvatureEffect = 0;
                        elseif j == 4
                            biElementaryCurvatureEffect = -k_peak;
                        else %last section
                            biElementaryCurvatureEffect = k_peak;
                        end
                    else
                        currLen = lastSectionLength;
                        if j == 1
                            biElementaryCurvatureEffect = problematicCurvatureEffect;
                        elseif j == 2
                            biElementaryCurvatureEffect = -problematicCurvatureEffect;
                        elseif j == 3 % 0 curvature part of bi elementary path
                            biElementaryCurvatureEffect = 0;
                        elseif j == 4
                            biElementaryCurvatureEffect = -problematicCurvatureEffect;
                        else %last section
                            biElementaryCurvatureEffect = problematicCurvatureEffect;
                        end
                    end
    
                    wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        -sigma * currLen + ...
                        biElementaryCurvatureEffect;
                    wayPoints(wayPointCounter + j).length = currLen;
                    
                    tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                        wayPoints(wayPointCounter + j - 1).tan, ...
                        wayPoints(wayPointCounter + j - 1).curv, ...
                        wayPoints(wayPointCounter + j).curv, ...
                        wayPoints(wayPointCounter + j).length, ...
                        0.01);
                    clothoidArray = [clothoidArray tempClothoid];
                    wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                    wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                end
                wayPointCounter = wayPointCounter + problematicIndex;
            end
    
    
        end
    else

        if(k0 > 0) % init curv is negative, single manuever is enough
            hcLength = -2 * theta0 / k0;
    
            % this arc is enough to compensate for heading and curvature
            % error
            positionCompensationLength = hcLength; % it may be equal to currArcLen?
            deltaY = positionError; 
            lambda = 0.5; 
            gamma = 0.5;
            [alpha,k_peak] = computeAlphak1...
            (positionCompensationLength,lambda,gamma,-deltaY,0);
            if(hcLength < currArcLen)% corrected before arc spline changes curvature
                % There will be as many waypoints as bi elementary curve
                % needs. The algorithm would probably never enter here.
    
                for j = 1:5 % 5 additional waypoints are needed
                    currLen = positionCompensationLength/8;
                    if j == 1
                        biElementaryCurvatureEffect = k_peak;
                    elseif j == 2
                        biElementaryCurvatureEffect = -k_peak;
                    elseif j == 3 % 0 curvature part of bi elementary path
                        currLen = positionCompensationLength/2;
                        biElementaryCurvatureEffect = 0;
                    elseif j == 4
                        biElementaryCurvatureEffect = -k_peak;
                    else %last section
                        biElementaryCurvatureEffect = k_peak;
                    end
    
                    wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        sigma * currLen + ...
                        biElementaryCurvatureEffect;
                    wayPoints(wayPointCounter + j).length = currLen;
                    
                    tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                        wayPoints(wayPointCounter + j - 1).tan, ...
                        wayPoints(wayPointCounter + j - 1).curv, ...
                        wayPoints(wayPointCounter + j).curv, ...
                        wayPoints(wayPointCounter + j).length, ...
                        0.01);
                    clothoidArray = [clothoidArray tempClothoid];
                    wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                    wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                end
                wayPointCounter = wayPointCounter + 5;
                break
            else % this arc is not enough, there will be residual error
                lenRatio = currArcLen / hcLength;
                curvatureRatioBiElementary = k_peak / (hcLength/8);
                if     (lenRatio > (7/8) ) % last part of bi-elementary is problematic
                    problematicIndex = 5;
                    lastSectionLength = currArcLen - 7/8*hcLength;
                elseif (lenRatio > (6/8) ) % 1 before last part is problematic
                    problematicIndex = 4;
                    lastSectionLength = currArcLen - 6/8*hcLength;
                elseif (lenRatio > (2/8) ) % 2 before last part is problematic
                    problematicIndex = 3;
                    lastSectionLength = currArcLen - 2/8*hcLength;
                elseif (lenRatio > (1/8) ) % 3 before last part is problematic
                    problematicIndex = 2;
                    lastSectionLength = currArcLen - 1/8*hcLength;
                else
                    problematicIndex = 1;
                    lastSectionLength = currArcLen;
                end
                problematicCurvatureEffect = curvatureRatioBiElementary * lastSectionLength;
    
                for j = 1:problematicIndex
                    if j ~= problematicIndex
                        currLen = positionCompensationLength/8;
                        if j == 1
                            biElementaryCurvatureEffect = k_peak;
                        elseif j == 2
                            biElementaryCurvatureEffect = -k_peak;
                        elseif j == 3 % 0 curvature part of bi elementary path
                            currLen = positionCompensationLength/2;
                            biElementaryCurvatureEffect = 0;
                        elseif j == 4
                            biElementaryCurvatureEffect = -k_peak;
                        else %last section
                            biElementaryCurvatureEffect = k_peak;
                        end
                    else
                        currLen = lastSectionLength;
                        if j == 1
                            biElementaryCurvatureEffect = problematicCurvatureEffect;
                        elseif j == 2
                            biElementaryCurvatureEffect = -problematicCurvatureEffect;
                        elseif j == 3 % 0 curvature part of bi elementary path
                            biElementaryCurvatureEffect = 0;
                        elseif j == 4
                            biElementaryCurvatureEffect = -problematicCurvatureEffect;
                        else %last section
                            biElementaryCurvatureEffect = problematicCurvatureEffect;
                        end
                    end
    
                    wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        sigma * currLen + ...
                        biElementaryCurvatureEffect;
                    wayPoints(wayPointCounter + j).length = currLen;
                    
                    tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                        wayPoints(wayPointCounter + j - 1).tan, ...
                        wayPoints(wayPointCounter + j - 1).curv, ...
                        wayPoints(wayPointCounter + j).curv, ...
                        wayPoints(wayPointCounter + j).length, ...
                        0.01);
                    clothoidArray = [clothoidArray tempClothoid];
                    wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                    wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                end
                wayPointCounter = wayPointCounter + problematicIndex;
            end
        else % single maneuver is not enough
            
            l1 = k0/sigma + sqrt(k0^2/2/(sigma^2) + theta0/sigma);  
            l2 = l1 - k0/sigma; 
            hcLength = l1 + l2;
    
            % this arc is enough to compensate for heading and curvature
            % error
            positionCompensationLength = hcLength; % it may be equal to currArcLen?
            deltaY = positionError; 
            lambda = 0.5; 
            gamma = 0.5;
            [alpha,k_peak] = computeAlphak1...
            (positionCompensationLength,lambda,gamma,-deltaY,0);
            
            sigmaChangingLenRatio = l1 / hcLength;
            if sigmaChangingLenRatio > (7/8)
                sigmaChangingIndex = 5;
                lengthUntilSigmaChange = l1 - (7/8)*hcLength;
            elseif sigmaChangingLenRatio > (6/8)
                sigmaChangingIndex = 4;
                lengthUntilSigmaChange = l1 - (6/8)*hcLength;
            elseif sigmaChangingLenRatio > (2/8)
                sigmaChangingIndex = 3;
                lengthUntilSigmaChange = l1 - (2/8)*hcLength;
            elseif sigmaChangingLenRatio > (1/8)
                sigmaChangingIndex = 2;
                lengthUntilSigmaChange = l1 - (1/8)*hcLength;
            else
                sigmaChangingIndex = 1;
                lengthUntilSigmaChange = l1;
            end
    
            if(hcLength < currArcLen) % corrected before arc spline changes curvature
                % There will be as many waypoints as bi elementary curve
                % needs and one more for the hc correction part.
                for j = 1:5 % 6 additional waypoints are needed
                    
                    currLen = positionCompensationLength/8;
                    if j == 1
                        biElementaryCurvatureEffect = k_peak;
                    elseif j == 2
                        biElementaryCurvatureEffect = -k_peak;
                    elseif j == 3 % 0 curvature part of bi elementary path
                        currLen = positionCompensationLength/2;
                        biElementaryCurvatureEffect = 0;
                    elseif j == 4
                        biElementaryCurvatureEffect = -k_peak;
                    else %last section
                        biElementaryCurvatureEffect = k_peak;
                    end
    
                    if(j == sigmaChangingIndex) % add a waypoint in the middle
                        partial_k_peak = k_peak * lengthUntilSigmaChange / currLen;
    
                        wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        (-sigma) * lengthUntilSigmaChange + ...
                        partial_k_peak;
                        wayPoints(wayPointCounter + j).length = lengthUntilSigmaChange;
                    
                        tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                            wayPoints(wayPointCounter + j - 1).tan, ...
                            wayPoints(wayPointCounter + j - 1).curv, ...
                            wayPoints(wayPointCounter + j).curv, ...
                            wayPoints(wayPointCounter + j).length, ...
                            0.01);
                        clothoidArray = [clothoidArray tempClothoid];
                        wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                        wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                        
                        wayPointCounter = wayPointCounter + 1;
    
                        % sigma = - sigma; % this changes sign at this point
                        residualLength = currLen - lengthUntilSigmaChange;
                        partial_residual_k_peak = k_peak * (residualLength) / currLen;
    
                        wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        sigma * residualLength + ...
                        partial_residual_k_peak;
                        wayPoints(wayPointCounter + j).length = residualLength;
                    
                        tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                            wayPoints(wayPointCounter + j - 1).tan, ...
                            wayPoints(wayPointCounter + j - 1).curv, ...
                            wayPoints(wayPointCounter + j).curv, ...
                            wayPoints(wayPointCounter + j).length, ...
                            0.01);
                        clothoidArray = [clothoidArray tempClothoid];
                        wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                        wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                        continue
                    end
    
                    wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        sigma * currLen + ...
                        biElementaryCurvatureEffect;
                    wayPoints(wayPointCounter + j).length = currLen;
                    
                    tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                        wayPoints(wayPointCounter + j - 1).tan, ...
                        wayPoints(wayPointCounter + j - 1).curv, ...
                        wayPoints(wayPointCounter + j).curv, ...
                        wayPoints(wayPointCounter + j).length, ...
                        0.01);
                    clothoidArray = [clothoidArray tempClothoid];
                    wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                    wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                end
                wayPointCounter = wayPointCounter + 5;
                break
            else % this arc is not enough, there will be residual error
                lenRatio = currArcLen / hcLength;
                curvatureRatioBiElementary = k_peak / (hcLength/8);
                if  (lenRatio > (7/8) ) % last part of bi-elementary is problematic
                    problematicIndex = 5;
                    lastSectionLength = currArcLen - 7/8*hcLength;
                elseif (lenRatio > (6/8) ) % 1 before last part is problematic
                    problematicIndex = 4;
                    lastSectionLength = currArcLen - 6/8*hcLength;
                elseif (lenRatio > (2/8) ) % 2 before last part is problematic
                    problematicIndex = 3;
                    lastSectionLength = currArcLen - 2/8*hcLength;
                elseif (lenRatio > (1/8) ) % 3 before last part is problematic
                    problematicIndex = 2;
                    lastSectionLength = currArcLen - 1/8*hcLength;
                else
                    problematicIndex = 1;
                    lastSectionLength = currArcLen;
                end
                problematicCurvatureEffect = curvatureRatioBiElementary * lastSectionLength;
    
                for j = 1:problematicIndex
                    if j ~= problematicIndex
                        currLen = positionCompensationLength/8;
                        if j == 1
                            biElementaryCurvatureEffect = k_peak;
                        elseif j == 2
                            biElementaryCurvatureEffect = -k_peak;
                        elseif j == 3 % 0 curvature part of bi elementary path
                            currLen = positionCompensationLength/2;
                            biElementaryCurvatureEffect = 0;
                        elseif j == 4
                            biElementaryCurvatureEffect = -k_peak;
                        else %last section
                            biElementaryCurvatureEffect = k_peak;
                        end
                    else
                        currLen = lastSectionLength;
                        if j == 1
                            biElementaryCurvatureEffect = problematicCurvatureEffect;
                        elseif j == 2
                            biElementaryCurvatureEffect = -problematicCurvatureEffect;
                        elseif j == 3 % 0 curvature part of bi elementary path
                            biElementaryCurvatureEffect = 0;
                        elseif j == 4
                            biElementaryCurvatureEffect = -problematicCurvatureEffect;
                        else %last section
                            biElementaryCurvatureEffect = problematicCurvatureEffect;
                        end
                    end
    
                    wayPoints(wayPointCounter + j).curv = ...
                        wayPoints(wayPointCounter + j - 1).curv + ...
                        sigma * currLen + ...
                        biElementaryCurvatureEffect;
                    wayPoints(wayPointCounter + j).length = currLen;
                    
                    tempClothoid = clothoid_v2(wayPoints(wayPointCounter + j - 1).pos,...
                        wayPoints(wayPointCounter + j - 1).tan, ...
                        wayPoints(wayPointCounter + j - 1).curv, ...
                        wayPoints(wayPointCounter + j).curv, ...
                        wayPoints(wayPointCounter + j).length, ...
                        0.01);
                    clothoidArray = [clothoidArray tempClothoid];
                    wayPoints(wayPointCounter + j).pos = [tempClothoid.allX(end) tempClothoid.allY(end)];
                    wayPoints(wayPointCounter + j).tan = tempClothoid.final_tan;
                end
                wayPointCounter = wayPointCounter + problematicIndex;
            end
    
    
        end
    end
    
end


end
