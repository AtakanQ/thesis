clear

load("all_clothoids.mat")
posError = 0.01; %meters
headingError = deg2rad(10); %radians
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

[clothoidArray,wayPoints] = ...
    fitArcSpline_v3([vehicleX vehicleY],vehicleTan,vehicleCurv,...
    clothoids_GT);


figure;
plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0 1 1],'DisplayName','Ground Truth')
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