set(groot, 'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on', 'defaultAxesZGrid', 'on');
addpath('../roadArcSpline/')

A = [0 0];
B = [15 10];
tzero = [1 0]';
tfinal = [sqrt(2) -sqrt(2)]';
init_curvature = 0;
final_curvature = .01;


trajectories = bezier(A,B,tzero,tfinal,init_curvature,final_curvature);

trajectories.plotCurves(1);
title("Bézier Curve Example","FontSize",13)
xlabel("xEast (m)","FontSize",13)
ylabel("yNorth (m)","FontSize",13)
axis equal
%% Plot varying P1

trajectories.plotCurvesByControlPoint(1);
legend();
 
%% Plot varying P2

trajectories.plotCurvesByControlPoint(2);
legend();

%% Plot varying P3

trajectories.plotCurvesByControlPoint(3);
legend();

%% Plot varying P4

trajectories.plotCurvesByControlPoint(4);
legend();
