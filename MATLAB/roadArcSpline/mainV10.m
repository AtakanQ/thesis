%This is created to get rural road segments only.

set(groot, 'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on', 'defaultAxesZGrid', 'on');
%% REAL DATA 
% AUTOBAHN 1 push
close all
clear
addpath('../../CLOTHOIDFITTING/G1fitting')

lat1 = 39.5900;
lat2 = 39.5023;
lon1 = 28.1720;
lon2 = 28.1870;
folderName = 'rural_road';
roadName = '1105231693'; 

[xEast, yNorth,number_of_roads,refLat,refLon] = ...
    retrieveOSM_v3(lat1, lat2, lon1, lon2, roadName,folderName);

[OSM_lats,OSM_lons,h] = enu2geodetic(xEast,yNorth,0,refLat,refLon,0,wgs84Ellipsoid);
set(groot, 'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on', 'defaultAxesZGrid', 'on');
figure;
geoplot(OSM_lats,OSM_lons,'LineWidth',2)
title("OpenStreetMap Data Example","FontSize",12)
geobasemap satellite

xEastCenter{1} = xEast;
yNorthCenter{1} = yNorth;

% Use clothoid fitting
for i = 1:length(xEastCenter)
    [theta_GT{i},curvature_GT{i},dk{i},L{i},...
        nevalG1,~,~,~,~] = ...
        G1spline( [xEastCenter{i} yNorthCenter{i}]);

    [all_clothoids{i}] = ...
        generateClothoids(xEastCenter{i},yNorthCenter{i},theta_GT{i},curvature_GT{i},dk{i},L{i});
end


figure;
for j = 1:numel(all_clothoids)
    for i = 1:numel(all_clothoids{j})
        [HERE_lats,HERE_lons,h] = enu2geodetic(all_clothoids{j}(i).allX,all_clothoids{j}(i).allY...
            ,0,refLat,refLon,0,wgs84Ellipsoid);
    
        geoplot(HERE_lats,HERE_lons,'LineWidth',2,'Color',j*[0, 0, 1/(numel(all_clothoids))])
        hold on
        title("Rural Road Example","FontSize",13)
        geobasemap satellite
    end
end

figure;
plot(xEastCenter{1},yNorthCenter{1},'*','MarkerSize',10,'Color',[0 1 0])
axis equal
hold on
for i = 1:numel(all_clothoids{1})
    plot(all_clothoids{1}(i).allX,all_clothoids{1}(i).allY,"Color",[1 0 0])
end
legend("Waypoints","G1 Fitted Road")

% pick some of the data from roads
% roadSegmentData = zeros(floor(numel(all_clothoids{1})/10),3);
% for i = 1:floor(numel(all_clothoids{1})/10)
%     roadSegmentData(i,1) = all_clothoids{1}(i+10).init_curv;
%     roadSegmentData(i,2) = all_clothoids{1}(i+10).final_curv;
%     roadSegmentData(i,3) = all_clothoids{1}(i+10).curv_length;
% end
% save("roadSegmentDataA1.mat","roadSegmentData")
%% Represent the road
lineCfg.lineDegreeDeviation = 0.2; % Allowed heading devation at the end of the segment (degrees)
lineCfg.rmsThreshold = 0.1; % RMS deviation from real road (meters)
% disp('This implementation has no lines!!!!!!!')
% lineCfg.rmsThreshold = -0.1; 
lineCfg.maximumAllowedDistance = 0.15; % Maximum deviation from real road (meters)
% lineCfg.numberOfPoints = 500; % Number of datapoints along the line segment

arcCfg.maximumDistance = 0.15; % Maximum allowed distance to deviate from real road.
arcCfg.initialTry = 5;
arcCfg.maximumNumArcs = 50;
% close all;
DEBUG = false;

for i = 1:length(xEastCenter)
    segments{i} = ...
        representRoad_v2(xEastCenter{i}(2:end-1),yNorthCenter{i}(2:end-1),...
        theta_GT{i}(2:end-1),curvature_GT{i}(2:end),...
        L{i}(2:end-1),all_clothoids{i}(2:end-1),lineCfg,arcCfg,DEBUG);
end

errorCfg.errorTol =  0.5;% percent. 
errorCfg.rmsError = 0.1; % Computed after concatenation
errorCfg.maxError = 0.2; % Computed after concatenation
errorCfg.headingDeviation = 2; % Degrees deviation allowed for concatenated lines

% [result_clothoids,concat_indices_clothoid,result_lines,concat_indices_line,mergedSegments] = ...
%     combineSegments(segments{1},all_clothoids{1}(2:end-1),errorCfg);


%% Generate other lanes
allX = [];
allY = [];
myX = [];
myY = [];
for j = 1:length(xEastCenter)
    allX{j} = [];
    allY{j} = [];
    for i = 2:(length(all_clothoids{j})-1)
        allX{j} = [allX{j}; all_clothoids{j}(i).allX'];
        allY{j} = [allY{j}; all_clothoids{j}(i).allY'];
    end
end

numFinalClothoids = 0;
numArcs = 0;
numLines = 0;
for j = 1:length(segments{1})
    if(segments{1}(j).type == "clothoid")
        numFinalClothoids = numFinalClothoids + 1;
        numArcs = numArcs + segments{1}(j).numArcs + 1;
        myX = [myX; segments{1}(j).allX'];
        myY = [myY; segments{1}(j).allY'];
    elseif(segments{1}(j).type == "line")
        numLines = numLines + 1;
        myX = [myX; segments{1}(j).allX];
        myY = [myY; segments{1}(j).allY]; 
    end
end

%% Plot the results

desiredNumElements = 100000;  % Replace with the desired number
desiredNumElements_GT = 100000;
for i = 1:numel(allX)
    downsamplingFactor = floor(numel(allX{i}) / desiredNumElements_GT);
    downsampledIndices = 1:downsamplingFactor:numel(allX{i});
    allX{i} = allX{i}(downsampledIndices);
    allY{i} = allY{i}(downsampledIndices);
end

% allX_OSM = [];
% allY_OSM = [];
% for i = 1:numel(all_clothoids_OSM)
%     allX_OSM = [allX_OSM all_clothoids_OSM(i).allX];
%     allY_OSM = [allY_OSM all_clothoids_OSM(i).allY];
% end

% downsamplingFactor = floor(numel(allX_OSM) / desiredNumElements);
% downsampledIndices = 1:downsamplingFactor:numel(allX_OSM);
% allX_OSM = allX_OSM(downsampledIndices);
% allY_OSM = allY_OSM(downsampledIndices);

figure
plot(allX{1},allY{1},'DisplayName','Ground Truth Reference','LineWidth',1.2)
hold on

plot(myX,myY,'DisplayName','Generated Reference Lane','LineWidth',1.2)
title("Real Road and Generated Road")
axis equal
xlabel("xEast (m)")
ylabel("yNorth (m)")
grid on
legend()



%% Compute distance between centers

% [rms_error_GT, max_error_GT, errors_GT] = computeSegmentError([allX{1}(1:10000) allY{1}(1:10000)],[allX{2}(1:10000) allY{2}(1:10000)]);
%% Compute memory usage

roadLen = 0;
numBytes = 0;
numArcSplines = 0;
numLineSegments = 0;
curvatures = [];
for i = 1:numel(segments{1})
    curvatures = [curvatures segments{1}(i).initialCurvature];
    if(segments{1}(i).numArcs>0)
        tempBytes = 16*segments{1}(i).numArcs;
        numArcSplines = numArcSplines + 1;
    else
        tempBytes = 16;
        numLineSegments = numLineSegments + 1;
    end
    roadLen = roadLen + segments{1}(i).segmentLength;
   numBytes = numBytes + tempBytes;  
end
%% Compute arc spline computation time





figure;
plot(curvatures,"LineWidth",1.2,"DisplayName","Curvature")
title("Road Segment Curvature","FontSize",13)
xlabel("Segment index","FontSize",13)
ylabel("Curvature (m^-^1)","FontSize",13)
legend();

numAllClothoid = numel(all_clothoids{1});
disp(['The road initially had ', num2str(numAllClothoid) ,' clothoids.'])
disp(['After approximation and combination of segments the road has ', num2str(numFinalClothoids),...
    ' clothoids and ', num2str(numArcs) , ' arcs were used.'])
disp(['Additionaly, ',num2str(numLines), ' lines were used.'])
disp(['Initially there were ', num2str(numAllClothoid) ' segments. After combination there are ', num2str(numFinalClothoids + numLines), ' segments.'])
save('last_work_V10')
