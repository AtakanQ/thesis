%% REAL DATA 
%inherited from V3
close all
% clear
addpath('../../CLOTHOIDFITTING/G1fitting')

lonlat = readCSV('..\..\PYTHON\O-21___4.csv');
refLat = mean(lonlat(:,2));
refLon = mean(lonlat(:,1));
[xEast, yNorth, zUp] = geodetic2enu(lonlat(:,2), ...
    lonlat(:,1), 0, refLat, refLon, 0, wgs84Ellipsoid);

figure;
plot(xEast,yNorth)
title('Real road')

[curvature_MVRC, centers_MVRC] = findCurvature([xEast yNorth]);

% Use clothoid fitting 
[theta,curvature_GT,dk,L,nevalG1,nevalF,iter,Fvalue,Fgradnorm] = G1spline( [xEast yNorth]);

centers_GT = findCenters([xEast yNorth], theta,curvature_GT);

% Generate clothoids.
[all_clothoids] = generateClothoids(xEast,yNorth,theta,curvature_GT,dk,L);


%% Generate arc segments
% turn left positive curvature

num_arc_points = 500;

%plotCircles_v2(curvature,centers,xEast,yNorth,num_arc_points,MVRC)
[arcSegments_MVRC] = plotCircles_v2(curvature_MVRC,centers_MVRC,xEast,yNorth,num_arc_points,true);

[arcSegments_GT] = plotCircles_v2(curvature_GT,centers_GT,xEast,yNorth,num_arc_points,false);

%Compute errors for each point along each segment.
%Additionally compute rms errors for each segment.
[errors_MVRC, rms_errors_MVRC] = computeErrors(arcSegments_MVRC,all_clothoids);
[errors_GT, rms_errors_GT] = computeErrors(arcSegments_GT,all_clothoids);

curvature_differences_MVRC = inspectArcs(curvature_MVRC,L,rms_errors_MVRC,true);
curvature_differences_GT = inspectArcs(curvature_GT,L,rms_errors_GT,false);


figure;
plot(curvature_MVRC)
hold on
plot(curvature_GT(2:end))
title('Curvature for each method')
legend('MVRC method','Clothoid fitting')
ylabel('Curvature')
xlabel('Segment index')


%% inspect a specific segment.
segment_idx = 16;

inspectSegment(segment_idx, curvature_MVRC, L, arcSegments_MVRC,...
    all_clothoids,errors_MVRC,xEast,yNorth,theta);
%% Represent the road
lineCfg.lineDegreeDeviation = 2; % Allowed heading devation at the end of the segment (degrees)
lineCfg.rmsThreshold = 0.2; % RMS deviation from real road (meters)
lineCfg.maximumAllowedDistance = 0.3; % Maximum deviation from real road (meters)
lineCfg.numberOfPoints = 500; % Number of datapoints along the line segment

arcCfg.maximumDistance = 0.1; % Maximum allowed distance to deviate from real road.
arcCfg.maximumNumArcs = 5;

segments = representRoad(xEast(2:end-1),yNorth(2:end-1),theta(2:end-1),curvature_MVRC,centers_MVRC,...
    L(2:end-1),all_clothoids(2:end-1),lineCfg,arcCfg);


% correlation_coefficient_length = corrcoef(L(1:end-1),rms_errors)
% correlation_coefficient_curvature_diff = corrcoef(curvature_differences,rms_errors(2:end))

