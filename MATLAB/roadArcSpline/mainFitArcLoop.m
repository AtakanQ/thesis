clear
close all
set(groot, 'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on', 'defaultAxesZGrid', 'on');

load("all_clothoids.mat")

clothoidIdx = 158;
clothoid_GT = all_clothoids{1}(clothoidIdx);
clothoids_GT = all_clothoids{1}(clothoidIdx:end);

laneWidth = 3.8;
shiftedCoords1 = [];
shiftedCoords2 = [];

for i = 1:numel(clothoids_GT)
    [tempShiftedCoords1, tempShiftedCoords2] = shiftCoordinates(...
        [clothoids_GT(i).allX' clothoids_GT(i).allY'], clothoids_GT(i).allTangent', laneWidth);
    shiftedCoords1 = [shiftedCoords1; tempShiftedCoords1];
    shiftedCoords2 = [shiftedCoords2; tempShiftedCoords2];
end
clear tempShiftedCoords1 tempShiftedCoords2



xyPairs = [];
allTangents = [];
allCurvatures = [];

for i = 1:numel(clothoids_GT)
    xyPairs = [xyPairs; clothoids_GT(i).allX' clothoids_GT(i).allY';];
    allTangents = [allTangents clothoids_GT(i).allTangent];
    allCurvatures = [allCurvatures clothoids_GT(i).allCurvature];
end

posErrors = [0.8 0.4 -0.4 -0.8];
headingErrors = deg2rad([4 2 -2 -4]);
curvatureErrors = [0.01 0.005 -0.005 -0.01];

arcSplineRMSPosErrors = zeros(numel(posErrors),numel(headingErrors),numel(curvatureErrors));
arcSplineMaxPosErrors = zeros(numel(posErrors),numel(headingErrors),numel(curvatureErrors));

clc
%% Try to fit clothoid

for i = 1:numel(posErrors)
    for j = 1:numel(headingErrors)
        for k = 1:numel(curvatureErrors)
            
            posError = posErrors(i);
            headingError = headingErrors(j);
            curvatureError = curvatureErrors(k);

            % Ego vehicle attitude
            vehicleTan = clothoid_GT.init_tan+headingError;
            vehicleX = clothoid_GT.allX(1) + posError * cos(clothoid_GT.init_tan + pi/2);
            vehicleY = clothoid_GT.allY(1) + posError * sin(clothoid_GT.init_tan + pi/2);
            vehicleCurv = clothoid_GT.init_curv + curvatureError;

            plotOn = false;
            % if k == 1 && j >2
            %     plotOn = true;
            % end
            % tic
            [clothoidArray,wayPoints] = ...
                fitArcSpline_v3([vehicleX vehicleY],vehicleTan,vehicleCurv,...
                clothoids_GT,plotOn,shiftedCoords1,shiftedCoords2,xyPairs);
            % timePassedClothoid = toc;
            arcSplineXY = [];
            for n = 1:numel(clothoidArray)
                arcSplineXY = [arcSplineXY; clothoidArray(n).allX' clothoidArray(n).allY'];
            end

            finalPosArc = [clothoidArray(end).allX(end) clothoidArray(end).allY(end)];
            finalTanArc = clothoidArray(end).final_tan;
            finalCurvArc = clothoidArray(end).final_curv;

            [closest_point, arcSplineClosestIndex] = findClosestPointOnLine(finalPosArc(1), finalPosArc(2),...
                finalTanArc + pi/2, xyPairs);

            [rms_error, max_error, ~] = computeSegmentError(arcSplineXY,xyPairs(1:arcSplineClosestIndex,:));

            arcSplineRMSPosErrors(i,j,k) = rms_error;
            arcSplineMaxPosErrors(i,j,k) = max_error;
            

            %print out final errors
            % arcSplineTangentError =  rad2deg(finalTanArc - allTangents(arcSplineClosestIndex));
            % arcSplineCurvatureError = finalCurvArc - allCurvatures(arcSplineClosestIndex);
        end
    end
end


