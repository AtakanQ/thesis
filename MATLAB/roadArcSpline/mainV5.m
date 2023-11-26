%% REAL DATA 
%inherited from V3
close all
clear
addpath('../../CLOTHOIDFITTING/G1fitting')


lonlat = readCSV('..\..\PYTHON\O-21___4.csv');
refLat = mean(lonlat(:,2));
refLon = mean(lonlat(:,1));
[xEast, yNorth, zUp] = geodetic2enu(lonlat(:,2), ...
    lonlat(:,1), 0, refLat, refLon, 0, wgs84Ellipsoid);

figure;
plot(xEast,yNorth)
title('Real road')

[curvature, centers] = findCurvature([xEast yNorth]);

diffxy = diff([xEast yNorth]);

% Use clothoid fitting 
[theta,k,dk,L,nevalG1,nevalF,iter,Fvalue,Fgradnorm] = G1spline( [xEast yNorth]);

% centers_test = findCenters([xEast yNorth], theta,k);

% Generate clothoids.
[all_clothoids] = generateClothoids(xEast,yNorth,theta,k,dk,L);


%% Test new methods
% Every clothoid has 420 = ( (clothoid_order + 1) * 20) data points in them.

% arcSegment_v3(center, startAngle, endAngle, radius,num_points)
% turn left positive curvature

% fitCircles(curvature,centers,theta)
num_arc_points = 500;


[arcSegments] = plotCircles_v2(curvature,centers,xEast,yNorth,num_arc_points);
% [arcSegments] = plotCircles_v2(k,centers_test,xEast,yNorth,num_arc_points);
hold on
plot(xEast,yNorth,'*','Color',[1 0 0])
% legend('Arcs','Road data points');

%Compute errors for each point along each segment.
%Additionally compute rms errors for each segment.
[errors, rms_errors] = computeError(arcSegments,all_clothoids);



figure;
first = 1;
plot(errors{first})
title(strcat('Error for curvature:',num2str(curvature(first)),...
    'Segment Length:  ', num2str(L(first))))
figure;
second = 10;
plot(errors{second})
title(strcat('Error for curvature:',num2str(curvature(second)),...
    'Segment Length:  ', num2str(L(second))))
figure;
third = 21;
plot(errors{third})
title(strcat('Error for curvature:',num2str(curvature(third)),...
    'Segment Length:  ', num2str(L(third))))