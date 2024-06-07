clear

load("all_clothoids.mat")
xError =  +0.2; %meters in x direction
yError = -0.01; %mteres is y direction
headingError = deg2rad(5); %radians
curvatureError = -0.01;
clothoid = all_clothoids{1}(158);

% Ego vehicle attitude
vehicleX = clothoid.allX(1)+xError;
vehicleY = clothoid.allY(1)+yError;
vehicleTan = clothoid.init_tan+headingError;
vehicleCurv = clothoid.init_curv + curvatureError;
%% Try to fit clothoid

[clothoidArray,middleArc] = fitArcSpline([vehicleX vehicleY],...
    vehicleTan,vehicleCurv,clothoid);

figure;
plot(clothoid.allX,clothoid.allY,'Color',[0 1 0],'DisplayName','Ground Truth')
hold on
axis equal
for i = 1:numel(clothoidArray)
    clothoidArray(i).plotPlain([i/7 0 0], "Clothoid num:" + num2str(i));
end
plot(middleArc.x_coor,middleArc.y_coor,"DisplayName","Arc","LineWidth",2)
legend();
%% Try to fit a Bézier curve

[trajectories] = fitBezier([vehicleX vehicleY],...
    vehicleTan,vehicleCurv,clothoid);

trajectories.plotCurves(5);
axis equal
hold on
plot(clothoid.allX,clothoid.allY,'Color',[0 1 0],'DisplayName','Ground Truth')
legend();

%% Plot both
trajectories.plotCurves(5);
axis equal
hold on
plot(clothoid.allX,clothoid.allY,'Color',[0 1 0],'DisplayName','Ground Truth')
for i = 1:numel(clothoidArray)
    clothoidArray(i).plotPlain([i/7 0 0], "Clothoid num:" + num2str(i));
end
plot(middleArc.x_coor,middleArc.y_coor,"DisplayName","Arc","LineWidth",2)
legend();

%% Plot varying P1

trajectories.plotCurvesByControlPoint(1);

plot(clothoid.allX,clothoid.allY,'Color',[0.5 0 0.5],'DisplayName','Lane Center','LineWidth',1.2)

legend();
 
%% Plot varying P2

trajectories.plotCurvesByControlPoint(2);

plot(clothoid.allX,clothoid.allY,'Color',[0.5 0 0.5],'DisplayName','Lane Center','LineWidth',1.2)

legend();

%% Plot varying P3

trajectories.plotCurvesByControlPoint(3);

plot(clothoid.allX,clothoid.allY,'Color',[0.5 0 0.5],'DisplayName','Lane Center','LineWidth',1.2)

legend();

%% Plot varying P4

trajectories.plotCurvesByControlPoint(4);

plot(clothoid.allX,clothoid.allY,'Color',[0.5 0 0.5],'DisplayName','Lane Center','LineWidth',1.2)

legend();