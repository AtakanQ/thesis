%% REAL DATA 
close all
clear
addpath('../../CLOTHOIDFITTING/G1fitting')

lat1 = 51.009334;
lat2 = 50.946392;
lon1 = 10.426450;
lon2 = 10.508444;
folderName = 'autobahn_4';
roadName = 'A 4';
HEREname = 'A4_laneData.mat';

[xEast, yNorth,number_of_roads,refLat,refLon] = ...
    retrieveOSM_v2(lat1, lat2, lon1, lon2, roadName,folderName);

[laneBorders,laneCenters] = retrieveHERE(folderName,HEREname,refLat,refLon);

xEast = laneCenters(2).xEast;
yNorth = laneCenters(2).yNorth;

figure;
for i = 1:size(laneCenters,2)    
    plot(laneCenters(i).xEast,laneCenters(i).yNorth,'DisplayName',strcat('Lane ',num2str(i)))
    hold on
    axis equal
end
plot(xEast,yNorth,'DisplayName','OSM')
legend()

if(folderName == 'autobahn_4') % there is some bug with this road
    xEast = xEast(15:end);
    yNorth = yNorth(15:end);
end

[curvature_MVRC, centers_MVRC, theta_MVRC] = findCurvature([xEast yNorth]);

% Use clothoid fitting 
[theta_GT,curvature_GT,dk,L,nevalG1,nevalF,iter,Fvalue,Fgradnorm] = G1spline( [xEast yNorth]);

centers_GT = findCenters([xEast yNorth], theta_GT,curvature_GT);

% Generate clothoids.
[all_clothoids] = generateClothoids(xEast,yNorth,theta_GT,curvature_GT,dk,L);


%% inspect a specific segment.
% segment_idx = 16;
% 
% inspectSegment(segment_idx, curvature_MVRC, L, arcSegments_MVRC,...
%     all_clothoids,errors_MVRC,xEast,yNorth,theta);
%% Represent the road
lineCfg.lineDegreeDeviation = 2; % Allowed heading devation at the end of the segment (degrees)
lineCfg.rmsThreshold = 0.1; % RMS deviation from real road (meters)
lineCfg.maximumAllowedDistance = 0.15; % Maximum deviation from real road (meters)
lineCfg.numberOfPoints = 500; % Number of datapoints along the line segment

arcCfg.maximumDistance = 0.15; % Maximum allowed distance to deviate from real road.
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

initial_segments.allX = [];
initial_segments.allY = [];
merged_segments.allX = [];
merged_segments.allY = [];
ground_truth.allX = [];
ground_truth.allY = [];
figure;
axis equal
for i = 1:length(segments)
    dim = size(segments(i).allX);
    if dim(2) == 1
        initial_segments.allX = [initial_segments.allX; segments(i).allX];
        initial_segments.allY = [initial_segments.allY; segments(i).allY];
    else
        initial_segments.allX = [initial_segments.allX; segments(i).allX'];
        initial_segments.allY = [initial_segments.allY; segments(i).allY'];
    end
end
for i = 1:length(mergedSegments)
    dim = size(mergedSegments(i).allX);
    if dim(2) == 1
        merged_segments.allX = [merged_segments.allX; mergedSegments(i).allX];
        merged_segments.allY = [merged_segments.allY; mergedSegments(i).allY];
    else
        merged_segments.allX = [merged_segments.allX;mergedSegments(i).allX'];
        merged_segments.allY = [merged_segments.allY; mergedSegments(i).allY'];
    end
end
for i = 1:length(all_clothoids)
    dim = size(all_clothoids(i).allX);
    if dim(2) == 1
        ground_truth.allX = [ground_truth.allX; all_clothoids(i).allX];
        ground_truth.allY = [ground_truth.allY; all_clothoids(i).allY];
    else
        ground_truth.allX = [ground_truth.allX; all_clothoids(i).allX'];
        ground_truth.allY = [ground_truth.allY; all_clothoids(i).allY'];
    end


end
plot(merged_segments.allX,merged_segments.allY,'Color',[0 0 1])
hold on
plot(initial_segments.allX,initial_segments.allY,'Color',[1 0 0])
hold on
plot(ground_truth.allX,ground_truth.allY,'Color',[0 1 0])
hold on
title("Road Representation Comparison")
xlabel("m")
ylabel("m")
legend('Merged segments','Initial segments','Ground truth');
axis equal
