%% REAL DATA 
close all
clear
addpath('../../CLOTHOIDFITTING/G1fitting')

lat1 = 50.9516;
lat2 = 50.9185;
lon1 = 11.1279;
lon2 = 11.2261;
folderName = 'autobahn_4';
roadName = 'A 4';

[xEast, yNorth,number_of_roads] = ...
    retrieveOSM_v2(lat1, lat2, lon1, lon2, roadName,folderName);

[curvature_MVRC, centers_MVRC, theta_MVRC] = findCurvature([xEast yNorth]);

% Use clothoid fitting 
[theta_GT,curvature_GT,dk,L,nevalG1,nevalF,iter,Fvalue,Fgradnorm] = G1spline( [xEast yNorth]);

centers_GT = findCenters([xEast yNorth], theta_GT,curvature_GT);

% Generate clothoids.
[all_clothoids] = generateClothoids(xEast,yNorth,theta_GT,curvature_GT,dk,L);


%% Generate arc segments
% turn left positive curvature

num_arc_points = 500;

figure;
plot(curvature_MVRC)
hold on
plot(curvature_GT(2:end))
title('Curvature for each method')
legend('MVRC method','Clothoid fitting')
ylabel('Curvature')
xlabel('Segment index')

%% inspect a specific segment.
% segment_idx = 16;
% 
% inspectSegment(segment_idx, curvature_MVRC, L, arcSegments_MVRC,...
%     all_clothoids,errors_MVRC,xEast,yNorth,theta);
%% Represent the road
lineCfg.lineDegreeDeviation = 2; % Allowed heading devation at the end of the segment (degrees)
lineCfg.rmsThreshold = 0.2; % RMS deviation from real road (meters)
lineCfg.maximumAllowedDistance = 0.3; % Maximum deviation from real road (meters)
lineCfg.numberOfPoints = 500; % Number of datapoints along the line segment

arcCfg.maximumDistance = 0.05; % Maximum allowed distance to deviate from real road.
arcCfg.initialTry = 5;
arcCfg.maximumNumArcs = 50;
% close all;
DEBUG = false;
% segments = representRoad_v2(xEast(2:end-1),yNorth(2:end-1),theta_MVRC,curvature_MVRC,centers_MVRC,...
%     L(2:end-1),all_clothoids(2:end-1),lineCfg,arcCfg,DEBUG);
segments = representRoad_v2(xEast(2:end-1),yNorth(2:end-1),theta_GT(2:end-1),curvature_GT(2:end),...
    L(2:end-1),all_clothoids(2:end-1),lineCfg,arcCfg,DEBUG);
% correlation_coefficient_length = corrcoef(L(1:end-1),rms_errors)
% correlation_coefficient_curvature_diff = corrcoef(curvature_differences,rms_errors(2:end))



figure;
temp_clothoids = all_clothoids(2:end-1);
for k = 1:length(segments)
    plot(segments(k).allX,segments(k).allY)
    hold on
    temp_clothoids(k).plotPlain()
end
title('Real Road and Generated Road')
grid on
axis equal

%% Try concatenating
errorCfg.errorTol =  0.5;% percent. 
errorCfg.rmsError = 0.2; % Computed after concatenation
errorCfg.maxError = 0.3; % Computed after concatenation
errorCfg.headingDeviation = 2; % Degrees deviation allowed for concatenated lines
% This parameter is the tolerance to decide while making the decision to
% concatenate. It is the percent tolerance between consecutive clothoids'
% derivatives.


% close all
figure;
plot(curvature_GT(2:end-1))
title('Ground Truth Curvature')
[result_clothoids,concat_indices_clothoid,result_lines,concat_indices_line,mergedSegments]...
    = combineSegments(segments,all_clothoids(2:end-1),errorCfg);