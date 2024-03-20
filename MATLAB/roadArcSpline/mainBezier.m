clear all
% close all

load('bezierData.mat')

segmentNumber = 158;
laneNumber = 1;

% segments{1}(1).type = 'clothoid';
% segments{1}(1).headingInitial = 0;
% segments{1}(1).headingFinal = 0;
% segments{1}(1).initialCurvature = 0;
% segments{1}(1).finalCurvature = 0;
% segments{1}(1).allX = linspace(0,10,1000);
% segments{1}(1).allY = segments{1}(1).allX.^2;

% add +-2 degrees error to tangents
init_tan = rad2deg(segments{laneNumber}(segmentNumber).headingInitial) + (rand-0.5) * 4;
final_tan = rad2deg(segments{laneNumber}(segmentNumber).headingFinal); %+ (rand-0.5) * 4;
tzero = [cosd(init_tan); sind(init_tan)];
tfinal = [cosd(final_tan); sind(final_tan)];

% add 5 percent error to curvatures
curvature_zero = segments{laneNumber}(segmentNumber).initialCurvature ;%* ( (rand-0.5)/10 + 1 );
curvature_final = segments{laneNumber}(segmentNumber).finalCurvature ; %* ( (rand-0.5)/10 + 1 );

% add +-20 centimeters error
initA = [segments{laneNumber}(segmentNumber).allX(1) segments{laneNumber}(segmentNumber).allY(1)];
initB = [segments{laneNumber}(segmentNumber).allX(end) segments{laneNumber}(segmentNumber).allY(end)];

pushA = [sin(segments{laneNumber}(segmentNumber).headingInitial) -cos(segments{laneNumber}(segmentNumber).headingInitial) ] .* ((rand-0.5)/5*10);
pushB = [ sin(segments{laneNumber}(segmentNumber).headingFinal) -cos(segments{laneNumber}(segmentNumber).headingFinal)] .* ((rand-0.5)/5*10);
pushB = 0;

A = initA + pushA;
B = initB + pushB;
trajectories = bezier(A,B,tzero,tfinal,curvature_zero,curvature_final);
trajectories.plotCurves();
hold on
plot(segments{laneNumber}(segmentNumber).allX,segments{laneNumber}(segmentNumber).allY,'--','Color',[1 0 0],'LineWidth',1)
trajectories.addVehicleDimensions(1.8,2.7,1)

[collisionArray] = checkCollision(trajectories,segments{laneNumber}(segmentNumber));

%% Plot everything


figure;

p1 = plot(trajectories.Curves{1}(:,1),trajectories.Curves{1}(:,2),'Color',[0 1 0],'DisplayName','Curve');
axis equal
hold on
p2 = plot(trajectories.Curves{1}(1,1),trajectories.Curves{1}(1,2),'*','MarkerSize',12,'Color',[0 0 1],'DisplayName','Start Point');
p3 = plot(trajectories.Curves{1}(end,1),trajectories.Curves{1}(end,2),'*','MarkerSize',12,'Color',[0 1 0],'DisplayName','End Point');
p4 = plot(trajectories.vehicleBoundaries.rearLeft{1}(:,1),trajectories.vehicleBoundaries.rearLeft{1}(:,2),'DisplayName','RearLeft');
p5 = plot(trajectories.vehicleBoundaries.rearRight{1}(:,1),trajectories.vehicleBoundaries.rearRight{1}(:,2),'DisplayName','RearRight');
p6 = plot(trajectories.vehicleBoundaries.frontLeft{1}(:,1),trajectories.vehicleBoundaries.frontLeft{1}(:,2),'DisplayName','FrontLeft') ;
p7 = plot(trajectories.vehicleBoundaries.frontRight{1}(:,1),trajectories.vehicleBoundaries.frontRight{1}(:,2),'DisplayName','FrontRight') ;
p8 = plot(segments{laneNumber}(segmentNumber).allX,segments{laneNumber}(segmentNumber).allY,'--','Color',[1 0 0],'LineWidth',1,'DisplayName','LaneCenter');

for i = 1:numel(all_clothoids_LB)

    for j = 1:numel(all_clothoids_LB{i})
        plot(all_clothoids_LB{i}(j).allX,all_clothoids_LB{i}(j).allY,'Color',[0 0 0],'LineWidth',2)

    end
end
legend([p1 p2 p3 p4 p5 p6 p7 p8]);
title('First Candidate Beziér Curve')

%% Generate arc-spline
arcSegClass = arcSegment;
init_pos = A;
init_tan = atan2(tzero(2),tzero(1));
init_curvature = curvature_zero;
final_curvature = curvature_final;
length = segments{laneNumber}(segmentNumber).segmentLength;
order = 5;
arc_spline = clothoid(init_pos,init_tan, init_curvature, final_curvature, length,order,arcSegClass);

figure;
axis equal
p1 = arc_spline.plotPlain();
hold on
p2 = plot(arc_spline.allX(1),arc_spline.allY(1),'*','MarkerSize',12,'Color',[0 0 1],'DisplayName','Start Point');
p3 = plot(arc_spline.allX(end),arc_spline.allY(end),'*','MarkerSize',12,'Color',[0 1 0],'DisplayName','End Point');
p4 = plot(segments{laneNumber}(segmentNumber).allX,segments{laneNumber}(segmentNumber).allY,'--','Color',[1 0 0],'LineWidth',1,'DisplayName','LaneCenter');
for i = 1:numel(all_clothoids_LB)

    for j = 1:numel(all_clothoids_LB{i})
        plot(all_clothoids_LB{i}(j).allX,all_clothoids_LB{i}(j).allY,'Color',[0 0 0],'LineWidth',2)

    end
end
legend([p1 p2 p3 p4]);
title('Arc Spline Curve')

%% Generate clothoid-like
myLength = deg2rad((final_tan - rad2deg(init_tan)) ) / (init_curvature/2 + final_curvature/2)

idx = 147;
myLength2 = (segments{1}(idx).headingFinal-segments{1}(idx).headingInitial) ...
    / (segments{1}(idx).initialCurvature/2 + segments{1}(idx).finalCurvature/2)
realLen = segments{1}(idx).segmentLength

% clothoid(init_pos,init_tan, init_curvature, final_curvature, length,order,arcSegClass)

arcSegClass = arcSegment;
init_pos = A;
init_tan = atan2(tzero(2),tzero(1));
init_curvature = curvature_zero;
final_curvature = curvature_final;
length = myLength;
order = 5;
arc_spline = clothoid(init_pos,init_tan, init_curvature, final_curvature, length,order,arcSegClass);

figure;
axis equal
p1 = arc_spline.plotPlain();
hold on
p2 = plot(arc_spline.allX(1),arc_spline.allY(1),'*','MarkerSize',12,'Color',[0 0 1],'DisplayName','Start Point');
p3 = plot(arc_spline.allX(end),arc_spline.allY(end),'*','MarkerSize',12,'Color',[0 1 0],'DisplayName','End Point');
p4 = plot(segments{laneNumber}(segmentNumber).allX,segments{laneNumber}(segmentNumber).allY,'--','Color',[1 0 0],'LineWidth',1,'DisplayName','LaneCenter');
for i = 1:numel(all_clothoids_LB)

    for j = 1:numel(all_clothoids_LB{i})
        plot(all_clothoids_LB{i}(j).allX,all_clothoids_LB{i}(j).allY,'Color',[0 0 0],'LineWidth',2)

    end
end
legend([p1 p2 p3 p4]);
title('Arc Spline Curve with More Accurate Length')
%% Try to fit clothoid

clothoid = all_clothoids{1}(158);
[clothoidArray] = fitArcSpline([clothoid.allX(1)+0.2 clothoid.allY(1)-0.01],clothoid.init_tan+deg2rad(1),clothoid.init_curv,clothoid)