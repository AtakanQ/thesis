clear
close all

load("all_clothoids.mat")
posError = 0.55; %meters
headingError = deg2rad(2); %radians
curvatureError = 0.01;
clothoid_GT = all_clothoids{1}(158);
clothoids_GT = all_clothoids{1}(158:end);

arcSegClass = arcSegment;
% clothoidApprox = clothoid([clothoid_GT.allX(1) clothoid_GT.allY(1)],...
%     clothoid_GT.init_tan , clothoid_GT.init_curv, clothoid_GT.final_curv,...
%                clothoid_GT.curv_length,6,arcSegClass);
% clothoid_GT.plotPlain()
% hold on
% clothoidApprox.plotPlain([1 0 0],'Approx')


% Ego vehicle attitude
vehicleTan = clothoid_GT.init_tan+headingError;
vehicleX = clothoid_GT.allX(1) + posError * cos(clothoid_GT.init_tan + pi/2);
vehicleY = clothoid_GT.allY(1) + posError * sin(clothoid_GT.init_tan + pi/2);
vehicleCurv = clothoid_GT.init_curv + curvatureError;
%% Try to fit clothoid

% Obsolete
% [clothoidArray,middleArc] = fitArcSpline([vehicleX vehicleY],...
%     vehicleTan,vehicleCurv,clothoid);

% [clothoidArray,wayPoints] = fitArcSpline_v2([vehicleX vehicleY],...
%     vehicleTan,vehicleCurv,clothoidApprox);
plotOn = false;
[clothoidArray,wayPoints] = ...
    fitArcSpline_v3([vehicleX vehicleY],vehicleTan,vehicleCurv,...
    clothoids_GT,plotOn);


figure;
plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0.5 0 1],'DisplayName','Ground Truth','LineWidth',1.2)
hold on
axis equal
for i = 1:numel(clothoidArray)
    clothoidArray(i).plotPlain([i/numel(clothoidArray) 0 0], "Clothoid num:" + num2str(i));
end
legend();

finalPosArc = [clothoidArray(end).allX(end) clothoidArray(end).allY(end)];
finalTanArc = clothoidArray(end).final_tan;
finalCurvArc = clothoidArray(end).final_curv;
xyPairs = [clothoid_GT.allX' clothoid_GT.allY'];

[closest_point, idx] = findClosestPointOnLine(finalPosArc(1), finalPosArc(2),...
    finalTanArc + pi/2, xyPairs);
%print out final errors
arcSplinePosError = norm( finalPosArc - [clothoid_GT.allX(idx) clothoid_GT.allY(idx)])
arcSplineTangentError =  rad2deg(finalTanArc - clothoid_GT.allTangent(idx))
arcSplineCurvatureError = finalCurvArc - clothoid_GT.allCurvature(idx)
%% Try to fit a Bézier curve

[trajectories] = fitBezier([vehicleX vehicleY],...
    vehicleTan,vehicleCurv,clothoid_GT);

trajectories.plotCurves(5);
axis equal
hold on
plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0 1 0],'DisplayName','Ground Truth')
legend();

%% Plot both
trajectories.plotCurves(5);
axis equal
hold on
plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0 1 0],'DisplayName','Ground Truth')
for i = 1:numel(clothoidArray)
    clothoidArray(i).plotPlain([i/numel(clothoidArray) 0 0], "Clothoid num:" + num2str(i));
end
% plot(middleArc.x_coor,middleArc.y_coor,"DisplayName","Arc","LineWidth",2)
legend();

%% Plot varying P1

trajectories.plotCurvesByControlPoint(1);

plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0.5 0 0.5],'DisplayName','Lane Center','LineWidth',1.2)

legend();
 
%% Plot varying P2

trajectories.plotCurvesByControlPoint(2);

plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0.5 0 0.5],'DisplayName','Lane Center','LineWidth',1.2)

legend();

%% Plot varying P3

trajectories.plotCurvesByControlPoint(3);

plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0.5 0 0.5],'DisplayName','Lane Center','LineWidth',1.2)

legend();

%% Plot varying P4

trajectories.plotCurvesByControlPoint(4);

plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0.5 0 0.5],'DisplayName','Lane Center','LineWidth',1.2)

legend();

%% Plot arc spline the results

% for i = 1:3
    ground_truth_xy = [clothoid_GT.allX' clothoid_GT.allY'];
    
% end

% plot the position error
arcSpline.allX = [];
arcSpline.allY = [];
arcSpline.allTangent = [];
arcSpline.allCurvature = [];
for i = 1:numel(clothoidArray)
    arcSpline.allX = [arcSpline.allX clothoidArray(i).allX];
    arcSpline.allY = [arcSpline.allY clothoidArray(i).allY];
    arcSpline.allTangent = [arcSpline.allTangent clothoidArray(i).allTangent];
    arcSpline.allCurvature = [arcSpline.allCurvature clothoidArray(i).allCurvature];
end
measurement_xy = [arcSpline.allX' arcSpline.allY'];
[~, ~, arcSplineErrors] = ...
    computeSegmentError(measurement_xy,ground_truth_xy);

figure;
plot(arcSpline.allX,arcSpline.allY,"LineWidth",1.5,"DisplayName","Arc Spline Trajectory")
hold on
plot(clothoid_GT.allX,clothoid_GT.allY,"LineWidth",1.5,"DisplayName","Road Centerline")
xlabel("xEast (m)","FontSize",13)
ylabel("yNorth (m)","FontSize",13)
grid on
title("Arc Spline Trajectory and Road Centerline","FontSize",13)
axis equal
legend();

figure;
plot(arcSplineErrors,"LineWidth",1.5,"DisplayName","Arc Spline Error")
xlabel("Sample index","FontSize",13)
ylabel("Distance to centerline (m)","FontSize",13)
grid on
title("Euclidian Distance Error of Arc Spline Trajectory","FontSize",13)
legend();

tangentsGT = clothoid_GT.allTangent(1:idx);
curvaturesGT = clothoid_GT.allCurvature(1:idx);
n1 = length(arcSpline.allTangent);
n2 = length(tangentsGT);

newIndices = linspace(1, n2, n1);
downsampledArrayTangent = interp1(1:n2, tangentsGT, newIndices);
downsampledArrayCurvature = interp1(1:n2, curvaturesGT, newIndices);
% degTan = rad2deg(downsampledArrayTangent);
% degTanArcspl = rad2deg(arcSpline.allTangent);
figure;
plot(rad2deg(downsampledArrayTangent-arcSpline.allTangent),"LineWidth",1.5,"DisplayName","Heading Error")
xlabel("Sample index","FontSize",13)
ylabel("Heading Error (°)","FontSize",13)
grid on
title("Heading Error of Arc Spline Trajectory","FontSize",13)
legend();

figure;
plot(downsampledArrayCurvature-arcSpline.allCurvature,"LineWidth",1.5,"DisplayName","Curvature Error")
xlabel("Sample index","FontSize",13)
ylabel("Curvature Error (m^-^1)","FontSize",13)
grid on
title("Curvature Error of Arc Spline Trajectory","FontSize",13)
legend();
%% Plot the Bézier results
% pick a trajectory
index = 225;
bezierTrajectory.allX = trajectories.Curves{index}(:,1);
bezierTrajectory.allY = trajectories.Curves{index}(:,2);
bezierTrajectory.allTangent = trajectories.Tangents{index};
bezierTrajectory.allCurvature = trajectories.Curvatures{index};

measurement_xy = [bezierTrajectory.allX bezierTrajectory.allY];
[~, ~, bezierErrors] = ...
    computeSegmentError(measurement_xy,ground_truth_xy);

figure;
plot(bezierTrajectory.allX,bezierTrajectory.allY,"LineWidth",1.5,"DisplayName","Bézier Trajectory")
hold on
plot(clothoid_GT.allX,clothoid_GT.allY,"LineWidth",1.5,"DisplayName","Road Centerline")
xlabel("xEast (m)","FontSize",13)
ylabel("yNorth (m)","FontSize",13)
grid on
title("Bézier Trajectory and Road Centerline","FontSize",13)
axis equal
legend();

figure;
plot(bezierErrors,"LineWidth",1.5,"DisplayName","Bézier Error")
xlabel("Sample index","FontSize",13)
ylabel("Distance to centerline (m)","FontSize",13)
grid on
title("Euclidian Distance Error of Bézier Trajectory","FontSize",13)
legend();

tangentsGT = clothoid_GT.allTangent;
curvaturesGT = clothoid_GT.allCurvature;
n1 = length(bezierTrajectory.allTangent);
n2 = length(tangentsGT);

newIndices = linspace(1, n2, n1);
downsampledArrayTangent = interp1(1:n2, tangentsGT, newIndices)';
downsampledArrayCurvature = interp1(1:n2, curvaturesGT, newIndices)';
% degTan = rad2deg(downsampledArrayTangent);
% degTanArcspl = rad2deg(arcSpline.allTangent);
figure;
plot(rad2deg(downsampledArrayTangent-bezierTrajectory.allTangent),"LineWidth",1.5,"DisplayName","Heading Error")
xlabel("Sample index","FontSize",13)
ylabel("Heading Error (°)","FontSize",13)
grid on
title("Heading Error of Bézier Trajectory","FontSize",13)
legend();

figure;
plot(downsampledArrayCurvature-bezierTrajectory.allCurvature,"LineWidth",1.5,"DisplayName","Curvature Error")
xlabel("Sample index","FontSize",13)
ylabel("Curvature Error (m^-^1)","FontSize",13)
grid on
title("Curvature Error of Bézier Trajectory","FontSize",13)
legend();

