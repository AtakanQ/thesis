clear
close all
set(groot, 'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on', 'defaultAxesZGrid', 'on');

load("all_clothoids.mat")
% load("all_clothoids_A1.mat")
load("all_clothoids_LB_A1.mat")


%case 1
% posError = 0.35; %meters
% headingError = deg2rad(2.0); %radians
% curvatureError = -0.001;
% fileName = "Case1";

%case 2
% posError = 0.20; %meters
% headingError = deg2rad(3); %radians
% curvatureError = 0.0015;
% fileName = "Case2";


%case 3
posError = -0.35; %meters
headingError = deg2rad(-4); %radians
curvatureError = -0.0009;
fileName = "Case3";

clothoidIdx = 55;
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


arcSegClass = arcSegment;


% Ego vehicle attitude
vehicleTan = clothoid_GT.init_tan+headingError;
vehicleX = clothoid_GT.allX(1) + posError * cos(clothoid_GT.init_tan + pi/2);
vehicleY = clothoid_GT.allY(1) + posError * sin(clothoid_GT.init_tan + pi/2);
vehicleCurv = clothoid_GT.init_curv + curvatureError;
% Plot the error and the road
figure;
plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0 1 0],'LineWidth',1.2)
hold on
axis equal
quiver(vehicleX,vehicleY,2.5*cos(vehicleTan),2.5*sin(vehicleTan),'Color',[1 0 0],'LineWidth',1.2)
plot(vehicleX,vehicleY,'*','MarkerSize',8,"LineWidth",1,'Color',[1 0 0])
plot(clothoid_GT.allX(1),clothoid_GT.allY(1),'*','MarkerSize',8,"LineWidth",1,'Color',[0 0 1])
quiver(clothoid_GT.allX(1),clothoid_GT.allY(1),2.5*cos(clothoid_GT.allTangent(1)),...
    2.5*sin(clothoid_GT.allTangent(1)),'LineWidth',1.2,'Color',[0 0 1])
plot([vehicleX, clothoid_GT.allX(1)], [vehicleY, clothoid_GT.allY(1)], '--', 'LineWidth', 1.2,'Color',[0 0 0]);
distance = sqrt((vehicleX - clothoid_GT.allX(1))^2 + (vehicleY - clothoid_GT.allY(1))^2);
midX = (vehicleX + clothoid_GT.allX(1)) / 2;
midY = (vehicleY + clothoid_GT.allY(1)) / 2;
text(midX, midY+0.2, sprintf('Distance: %.2f m', distance), ...
    'FontSize', 10, 'Color', [0 0 0], 'HorizontalAlignment', 'center',...
    'VerticalAlignment','bottom');

text(vehicleX+0.1, vehicleY-0.2, sprintf('Heading Angle: %.2f°', rad2deg(vehicleTan)), ...
    'FontSize', 10, 'Color', [0 0 0], 'HorizontalAlignment', 'left');

text(clothoid_GT.allX(1)-.1,clothoid_GT.allY(1)-0.2, sprintf('Heading Angle: %.2f°', rad2deg(clothoid_GT.allTangent(1))), ...
    'FontSize', 10, 'Color', [0 0 0], 'HorizontalAlignment', 'right');
title("Road Waypoint and Ego Vehicle Attitude","FontSize",13)
xlabel("xEast (m)","FontSize",13)
ylabel("yNorth (m)","FontSize",13)

%lane boundaries
h4 = plot(shiftedCoords1(:,1),shiftedCoords1(:,2),'--','Color',[0 0 0],'DisplayName','Lane Boundary','LineWidth',1);
plot(shiftedCoords2(:,1),shiftedCoords2(:,2),'--','Color',[0 0 0],'LineWidth',1)

% Plot dummy points for the legend
h1 = plot(nan, nan, 'Color',[0 1 0], 'LineWidth', 1.2);
h2 = plot(nan, nan, 'Color',[1 0 0], 'LineWidth', 1.2);
h3 = plot(nan, nan, 'Color',[0 0 1], 'LineWidth', 1.2);

% Create a legend with only the dummy points
legend([h1, h2, h3,h4], {'Road Centerline', 'Initial Waypoint', 'Ego Vehicle','Lane Boundary'}, 'Location', 'Best');


%% Try to fit clothoid
xyPairs = [];
allTangents = [];
allCurvatures = [];

for i = 1:numel(clothoids_GT)
    xyPairs = [xyPairs; clothoids_GT(i).allX' clothoids_GT(i).allY';];
    allTangents = [allTangents clothoids_GT(i).allTangent];
    allCurvatures = [allCurvatures clothoids_GT(i).allCurvature];
end
% Obsolete
% [clothoidArray,middleArc] = fitArcSpline([vehicleX vehicleY],...
%     vehicleTan,vehicleCurv,clothoid);

% [clothoidArray,wayPoints] = fitArcSpline_v2([vehicleX vehicleY],...
%     vehicleTan,vehicleCurv,clothoidApprox);
plotOn = false;
% profile on
tic
[clothoidArray,wayPoints] = ...
    fitArcSpline_v3([vehicleX vehicleY],vehicleTan,vehicleCurv,...
    clothoids_GT,plotOn,shiftedCoords1,shiftedCoords2,xyPairs);
timePassedClothoid = toc
% profile viewer
% profile off

figure;
h = plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0.5 0 1],'DisplayName','Ground Truth','LineWidth',1.2);
hold on
axis equal
for i = 2:numel(clothoids_GT)
    plot(clothoids_GT(i).allX,clothoids_GT(i).allY,'Color',[0.5 0 1],'DisplayName','Ground Truth','LineWidth',1.2);
end

for i = 1:numel(clothoidArray)
    hT = clothoidArray(i).plotPlain([i/numel(clothoidArray) 0 0], "Clothoid num:" + num2str(i));
    h = [h hT];
end
% h3 = plotLaneBoundaries(all_clothoids_LB, LB_indices);
h3 = plot(shiftedCoords1(:,1),shiftedCoords1(:,2),'--','Color',[0 0 0],'DisplayName','Lane Boundary','LineWidth',1);
plot(shiftedCoords2(:,1),shiftedCoords2(:,2),'--','Color',[0 0 0],'LineWidth',1)
h = [h h3];
legend(h);

finalPosArc = [clothoidArray(end).allX(end) clothoidArray(end).allY(end)];
finalTanArc = clothoidArray(end).final_tan;
finalCurvArc = clothoidArray(end).final_curv;


[closest_point, arcSplineClosestIndex] = findClosestPointOnLine(finalPosArc(1), finalPosArc(2),...
    finalTanArc + pi/2, xyPairs);
%print out final errors
arcSplinePosError = norm( finalPosArc - closest_point);
arcSplineTangentError =  rad2deg(finalTanArc - allTangents(arcSplineClosestIndex));
arcSplineCurvatureError = finalCurvArc - allCurvatures(arcSplineClosestIndex);


%% Plot arc spline the results
close all
ground_truth_xy = xyPairs;
    
% plot the position error
arcSpline.allX = [];
arcSpline.allY = [];
arcSpline.allTangent = [];
arcSpline.allCurvature = [];
arcSpline.length = 0;
for i = 1:numel(clothoidArray)
    arcSpline.allX = [arcSpline.allX clothoidArray(i).allX];
    arcSpline.allY = [arcSpline.allY clothoidArray(i).allY];
    arcSpline.allTangent = [arcSpline.allTangent clothoidArray(i).allTangent];
    arcSpline.allCurvature = [arcSpline.allCurvature clothoidArray(i).allCurvature];
    arcSpline.length = arcSpline.length + clothoidArray(i).curv_length;
end
measurement_xy = [arcSpline.allX' arcSpline.allY'];
[~, ~, arcSplineErrors] = ...
    computeSegmentError(measurement_xy,ground_truth_xy);

figure;
h1 = plot(arcSpline.allX,arcSpline.allY,"LineWidth",1.5,"DisplayName","Arc Spline Trajectory");
hold on
h2 = plot(ground_truth_xy(:,1),ground_truth_xy(:,2),"LineWidth",1.5,"DisplayName","Road Centerline");
h3 = plot(shiftedCoords1(:,1),shiftedCoords1(:,2),'--','Color',[0 0 0],'DisplayName','Lane Boundary','LineWidth',1);
plot(shiftedCoords2(:,1),shiftedCoords2(:,2),'--','Color',[0 0 0],'LineWidth',1)
xlabel("xEast (m)","FontSize",13)
ylabel("yNorth (m)","FontSize",13)
grid on
title("Arc Spline Trajectory and Road Centerline","FontSize",13)
axis equal
minY = min(arcSpline.allY([1 length(arcSpline.allY)]));
maxY = max(arcSpline.allY([1 length(arcSpline.allY)]));
ylim([minY-5 maxY+5])

legend([h1 h2 h3]);

xAxisArcSpline = 10*(0:0.01:(length(arcSplineErrors)-1)/100 );
figure;
plot(xAxisArcSpline,arcSplineErrors,"LineWidth",1.5,"DisplayName","Arc Spline Error")
xlabel("Trajectory Length (m)","FontSize",13)
ylabel("Distance to centerline (m)","FontSize",13)
grid on
title("Euclidian Distance Error of Arc Spline Trajectory","FontSize",13)
legend();

tangentsGT = allTangents(1:arcSplineClosestIndex);
curvaturesGT = allCurvatures(1:arcSplineClosestIndex);
n1 = length(arcSpline.allTangent);
n2 = length(tangentsGT);

newIndices = linspace(1, n2, n1);
downsampledArrayTangent = interp1(1:n2, tangentsGT, newIndices);
downsampledArrayCurvature = interp1(1:n2, curvaturesGT, newIndices);
% degTan = rad2deg(downsampledArrayTangent);
% degTanArcspl = rad2deg(arcSpline.allTangent);
arcSplineHeadingErrors = rad2deg(downsampledArrayTangent-arcSpline.allTangent);
figure;
plot(xAxisArcSpline,arcSplineHeadingErrors,"LineWidth",1.5,"DisplayName","Heading Error")
xlabel("Trajectory Length (m)","FontSize",13)
ylabel("Heading Error (°)","FontSize",13)
grid on
title("Heading Error of Arc Spline Trajectory","FontSize",13)
legend();

arcSplineCurvatureErrors = downsampledArrayCurvature-arcSpline.allCurvature;
figure;
plot(xAxisArcSpline,arcSplineCurvatureErrors,"LineWidth",1.5,"DisplayName","Curvature Error")
xlabel("Trajectory Length (m)","FontSize",13)
ylabel("Curvature Error (m^-^1)","FontSize",13)
grid on
title("Curvature Error of Arc Spline Trajectory","FontSize",13)
legend();

%% Simulate further

nextInitPos = [clothoidArray(end).allX(end) clothoidArray(end).allY(end)];
nextInitTan = clothoidArray(end).allTangent(end);
nextInitCurv = clothoidArray(end).allCurvature(end);

passedLength = arcSplineClosestIndex/100;
totalLength = 0 ;

for i = 1:numel(clothoids_GT)
    if passedLength < totalLength
        last_idx = i-1;
        remainingLength = totalLength - passedLength;
        break
    end
    totalLength = totalLength + clothoids_GT(i).curv_length;
end


tempClothoid = clothoid_v2(xyPairs(arcSplineClosestIndex,:),allTangents(arcSplineClosestIndex),...
    allCurvatures(arcSplineClosestIndex),clothoids_GT(last_idx).final_curv,remainingLength,0.01);

clothoids_GT_next = [tempClothoid clothoids_GT((last_idx+1):end)];

plotOn = true;
[clothoidArrayRoadFollow] = ...
    followRoad(nextInitPos,nextInitTan,nextInitCurv,...
    clothoids_GT_next,plotOn,shiftedCoords1,shiftedCoords2,2);

clothoidArray = [clothoidArray clothoidArrayRoadFollow'];

%% plot overall
close all
figure;
h2 = plot(ground_truth_xy(:,1),ground_truth_xy(:,2),'Color',[1 0 0],"LineWidth",1.0,"DisplayName","Road Centerline");
hold on
for i = 1:numel(clothoidArray)
    h1 = clothoidArray(i).plotPlain([0 0 1],"Arc-spline Trajectory");
    h1.LineWidth = 1.5;
end
h3 = plot(shiftedCoords1(:,1),shiftedCoords1(:,2),'--','Color',[0 0 0],'DisplayName','Lane Boundary','LineWidth',1);
plot(shiftedCoords2(:,1),shiftedCoords2(:,2),'--','Color',[0 0 0],'LineWidth',1)
xlabel("xEast (m)","FontSize",13)
ylabel("yNorth (m)","FontSize",13)
grid on
title("Arc Spline Trajectory and Road Centerline","FontSize",13)
legend([h1 h2 h3])

save("mainFitArc_v2"+fileName+".mat","clothoidArray","clothoidArrayRoadFollow")

%% Plot arc spline the results
finalPosArc = [clothoidArray(end).allX(end) clothoidArray(end).allY(end)];
finalTanArc = clothoidArray(end).final_tan;
finalCurvArc = clothoidArray(end).final_curv;


[closest_point, arcSplineClosestIndex] = findClosestPointOnLine(finalPosArc(1), finalPosArc(2),...
    finalTanArc + pi/2, xyPairs);

ground_truth_xy = xyPairs;
    
% plot the position error
arcSpline.allX = [];
arcSpline.allY = [];
arcSpline.allTangent = [];
arcSpline.allCurvature = [];
arcSpline.length = 0;
% figure;
for i = 1:numel(clothoidArray)
% for i = 10:11
    arcSpline.allX = [arcSpline.allX clothoidArray(i).allX];
    arcSpline.allY = [arcSpline.allY clothoidArray(i).allY];
    arcSpline.allTangent = [arcSpline.allTangent clothoidArray(i).allTangent];
    arcSpline.allCurvature = [arcSpline.allCurvature clothoidArray(i).allCurvature];
    arcSpline.length = arcSpline.length + clothoidArray(i).curv_length;
    % clothoidArray(i).plotPlain([0 0 i/11]);
    % hold on
    % axis equal
end
measurement_xy = [arcSpline.allX' arcSpline.allY'];
[~, ~, arcSplineErrors] = ...
    computeSegmentError(measurement_xy,ground_truth_xy);

figure;
h2 = plot(ground_truth_xy(:,1),ground_truth_xy(:,2),"LineWidth",1.0,"DisplayName","Road Centerline");
hold on
h1 = plot(arcSpline.allX,arcSpline.allY,"LineWidth",1.5,"DisplayName","Arc Spline Trajectory");
h3 = plot(shiftedCoords1(:,1),shiftedCoords1(:,2),'--','Color',[0 0 0],'DisplayName','Lane Boundary','LineWidth',1);
plot(shiftedCoords2(:,1),shiftedCoords2(:,2),'--','Color',[0 0 0],'LineWidth',1)
xlabel("xEast (m)","FontSize",13)
ylabel("yNorth (m)","FontSize",13)
grid on
title("Arc Spline Trajectory and Road Centerline","FontSize",13)
axis equal
minY = min(arcSpline.allY([1 length(arcSpline.allY)]));
maxY = max(arcSpline.allY([1 length(arcSpline.allY)]));
ylim([minY-5 maxY+5])

legend([h1 h2 h3]);

xAxisArcSpline = (0:0.01:(length(arcSplineErrors)-1)/100 );
figure;
plot(xAxisArcSpline,arcSplineErrors,"LineWidth",1.5,"DisplayName","Arc Spline Error")
xlabel("Trajectory Length (m)","FontSize",13)
ylabel("Distance to centerline (m)","FontSize",13)
grid on
title("Euclidian Distance Error of Arc Spline Trajectory","FontSize",13)
legend();

tangentsGT = allTangents(1:arcSplineClosestIndex);
curvaturesGT = allCurvatures(1:arcSplineClosestIndex);
n1 = length(arcSpline.allTangent);
n2 = length(tangentsGT);

newIndices = linspace(1, n2, n1);
downsampledArrayTangent = interp1(1:n2, tangentsGT, newIndices);
downsampledArrayCurvature = interp1(1:n2, curvaturesGT, newIndices);
% degTan = rad2deg(downsampledArrayTangent);
% degTanArcspl = rad2deg(arcSpline.allTangent);
arcSplineHeadingErrors = rad2deg(downsampledArrayTangent-arcSpline.allTangent);
figure;
plot(xAxisArcSpline,arcSplineHeadingErrors,"LineWidth",1.5,"DisplayName","Heading Error")
xlabel("Trajectory Length (m)","FontSize",13)
ylabel("Heading Error (°)","FontSize",13)
grid on
title("Heading Error of Arc Spline Trajectory","FontSize",13)
legend();

arcSplineCurvatureErrors = downsampledArrayCurvature-arcSpline.allCurvature;
figure;
plot(xAxisArcSpline,arcSplineCurvatureErrors,"LineWidth",1.5,"DisplayName","Curvature Error")
xlabel("Trajectory Length (m)","FontSize",13)
ylabel("Curvature Error (m^-^1)","FontSize",13)
grid on
title("Curvature Error of Arc Spline Trajectory","FontSize",13)
legend();

