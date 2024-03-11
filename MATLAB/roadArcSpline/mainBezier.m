clear all
% close all

load('bezierData.mat')

segmentNumber = 1;
laneNumber = 1;

segments{1}(1).type = 'clothoid';
segments{1}(1).headingInitial = 0;
segments{1}(1).headingFinal = 0;
segments{1}(1).initialCurvature = 0;
segments{1}(1).finalCurvature = 0;
segments{1}(1).allX = linspace(0,10,1000);
segments{1}(1).allY = segments{1}(1).allX.^2;

% add +-2 degrees error to tangents
init_tan = rad2deg(segments{laneNumber}(segmentNumber).headingInitial) + (rand-0.5) * 4;
final_tan = rad2deg(segments{laneNumber}(segmentNumber).headingFinal) + (rand-0.5) * 4;
tzero = [cosd(init_tan); sind(init_tan)];
tfinal = [cosd(final_tan); sind(final_tan)];

% add 5 percent error to curvatures
curvature_zero = segments{laneNumber}(segmentNumber).initialCurvature * ( (rand-0.5)/10 + 1 );
curvature_final = segments{laneNumber}(segmentNumber).finalCurvature * ( (rand-0.5)/10 + 1 );

% add +-20 centimeters error
initA = [segments{laneNumber}(segmentNumber).allX(1) segments{laneNumber}(segmentNumber).allY(1)];
initB = [segments{laneNumber}(segmentNumber).allX(end) segments{laneNumber}(segmentNumber).allY(end)];

pushA = [sin(segments{laneNumber}(segmentNumber).headingInitial) -cos(segments{laneNumber}(segmentNumber).headingInitial) ] .* ((rand-0.5)/5*2);
pushB = [ sin(segments{laneNumber}(segmentNumber).headingFinal) -cos(segments{laneNumber}(segmentNumber).headingFinal)] .* ((rand-0.5)/5*2);

A = initA + pushA;
B = initB + pushB;
trajectories = bezier(A,B,tzero,tfinal,curvature_zero,curvature_final);
trajectories.plotCurves();
hold on
plot(segments{laneNumber}(segmentNumber).allX,segments{laneNumber}(segmentNumber).allY,'Color',[1 0 0],'LineWidth',1)
trajectories.addVehicleDimensions(1.5,4,0.5)

[collisionArray] = checkCollision(trajectories,segments{laneNumber}(segmentNumber));