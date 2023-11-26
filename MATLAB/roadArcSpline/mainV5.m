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

curvature_differences = abs(diff(curvature));

figure;
plot(curvature_differences,'Color',[0 0 1])
hold on
ylabel('Curvature difference')
yyaxis right
plot(rms_errors(2:end),'Color',[1 0 0])
ylabel('RMS error')
legend('Abs Curvature differences', 'RMS errors');
title('Absolute Curvature Differences and RMS Errors')
grid on
xlabel('Segment index')
% inspect a specific segment.
segment_idx = 36;
figure;
plot(errors{segment_idx})
title(strcat('Error for radius:',num2str(abs( 1/curvature(segment_idx)) ),...
    'Segment Length:  ', num2str(L(segment_idx))))

figure;
arcSegments{segment_idx}.plotArc();
hold on
all_clothoids(segment_idx).plotPlain();
title('Generated Arc and Clothoid for Specified Segment')
axis equal
heading_change_for_specified_index = rad2deg(all_clothoids(segment_idx).final_tan -...
    all_clothoids(segment_idx).init_tan)



