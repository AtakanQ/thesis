set(groot, 'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on', 'defaultAxesZGrid', 'on');
%% REAL DATA 
% AUTOBAHN 1
close all
clear
addpath('../../CLOTHOIDFITTING/G1fitting')

lat1 = 51.2632;
lat2 = 51.1882;
lon1 = 7.2180;
lon2 = 7.2579;
folderName = 'autobahn_1';
roadName = 'A 1'; 
HEREname = 'A1_LaneDataRaw.mat';

[xEast, yNorth,number_of_roads,refLat,refLon] = ...
    retrieveOSM_v2(lat1, lat2, lon1, lon2, roadName,folderName);

[OSM_lats,OSM_lons,h] = enu2geodetic(xEast,yNorth,0,refLat,refLon,0,wgs84Ellipsoid);
set(groot, 'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on', 'defaultAxesZGrid', 'on');
% figure;
% geoplot(OSM_lats,OSM_lons,'LineWidth',2)
% title("OpenStreetMap Data Example","FontSize",12)
% geobasemap satellite

[laneBorders,laneCenters] = retrieveHERE_v3(folderName,HEREname,refLat,refLon,[lat1 lat2], [lon1 lon2]);

%Use left most lane
laneCntr = 1;
for j = 9:11 %8-9-10 seems feasible
    xEastCenter{laneCntr} = laneCenters(j).xEast(laneCenters(j).lats > 51.2014);
    yNorthCenter{laneCntr} = laneCenters(j).yNorth(laneCenters(j).lats > 51.2014);
    laneCntr = laneCntr + 1;
end

for j = 8:13
    xEastCenter_LB{j} = laneBorders(j).xEast;
    yNorthCenter_LB{j} = laneBorders(j).yNorth;
end

figure;
for i = 1:size(xEastCenter,2)
    xEastCenter{i} =xEastCenter{i}(1:(length(xEastCenter{i})/2)); % it is flipped
    yNorthCenter{i} =yNorthCenter{i}(1:(length(yNorthCenter{i})/2)); % it is flipped
    plot(xEastCenter{i},yNorthCenter{i},'DisplayName',strcat('Lane ',num2str(i)))
    hold on
    axis equal
end
title("Here Maps Road")
grid on
% plot(xEast,yNorth,'DisplayName','OSM')
legend()

if(strcmp(folderName , "autobahn_4")) % there is some bug with this road
    xEast = xEast(15:end);
    yNorth = yNorth(15:end);
end

% Use clothoid fitting
compTimeClothoid = [];
for i = 1:length(xEastCenter)
    [theta_GT{i},curvature_GT{i},dk{i},L{i},...
        nevalG1,~,~,~,~] = ...
        G1spline( [xEastCenter{i} yNorthCenter{i}]);
    tic
    [all_clothoids{i}] = ...
        generateClothoids(xEastCenter{i},yNorthCenter{i},theta_GT{i},curvature_GT{i},dk{i},L{i});
    compTimeClothoid = [compTimeClothoid toc];
end
compTimeClothoid

figure;
for j = 1:numel(all_clothoids)
    for i = 1:numel(all_clothoids{j})
        [HERE_lats,HERE_lons,h] = enu2geodetic(all_clothoids{j}(i).allX,all_clothoids{j}(i).allY...
            ,0,refLat,refLon,0,wgs84Ellipsoid);
    
        geoplot(HERE_lats,HERE_lons,'LineWidth',2,'Color',j*[0, 0, 1/(numel(all_clothoids))])
        hold on
        title("G1 Clothoid Fitted Road (Autobahn 1)","FontSize",13)
        geobasemap satellite
    end
end

% Use clothoid fitting to fit lane boundaries
% for i = 2:length(xEastCenter_LB)
%     [theta_GT_LB{i},curvature_GT_LB{i},dk_LB{i},L_LB{i},...
%         nevalG1_LB,~,~,~,~] = ...
%         G1spline( [xEastCenter_LB{i} yNorthCenter_LB{i}]);
% 
%     [all_clothoids_LB{i}] = ...
%         generateClothoids(xEastCenter_LB{i},yNorthCenter_LB{i},theta_GT_LB{i},curvature_GT_LB{i},dk_LB{i},L_LB{i});
% end

figure;
plot(xEastCenter{1},yNorthCenter{1},'*','MarkerSize',10,'Color',[0 1 0])
axis equal
hold on
for i = 1:numel(all_clothoids{1})
    plot(all_clothoids{1}(i).allX,all_clothoids{1}(i).allY,"Color",[1 0 0])
end
legend("Waypoints","G1 Fitted Road")

[theta_OSM,curvature_OSM,dk_OSM,L_OSM,...
    ~,~,~,~,~] = ...
    G1spline( [xEast yNorth]);
all_clothoids_OSM = generateClothoids(xEast,yNorth,...
    theta_OSM,curvature_OSM,dk_OSM,L_OSM);

% pick some of the data from roads
roadSegmentData = zeros(floor(numel(all_clothoids{1})/10),3);
for i = 1:floor(numel(all_clothoids{1})/10)
    roadSegmentData(i,1) = all_clothoids{1}(i+10).init_curv;
    roadSegmentData(i,2) = all_clothoids{1}(i+10).final_curv;
    roadSegmentData(i,3) = all_clothoids{1}(i+10).curv_length;
end
save("roadSegmentDataA1.mat","roadSegmentData")
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




% segments{1} = mergedSegments;
%% Test
% figure;
% for i = 2:(length(all_clothoids{3})-1)
%     plot(all_clothoids{3}(i).allX,all_clothoids{3}(i).allY,'Color',[0 0 1])
%     hold on
%     if(segments{3}(i-1).type == "clothoid")
%         plot(segments{3}(i-1).allX,segments{3}(i-1).allY,'--','Color',[1 0 0])
%     else
%         plot(segments{3}(i-1).allX,segments{3}(i-1).allY,'--','Color',[0 1 0])
%     end
%     axis equal
% end

%% Generate other lanes
laneWidth = -3.52; % meters
numLanes = 2; % other lanes
[otherLanes,xEastShifted,yNorthShifted] = generateOtherLanes(segments{1},laneWidth,numLanes);
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
for i = 1:numel(xEastShifted)
    downsamplingFactor = floor(numel(xEastShifted{i}) / desiredNumElements);
    downsampledIndices = 1:downsamplingFactor:numel(xEastShifted{i});
    xEastShifted{i} = xEastShifted{i}(downsampledIndices);
    yNorthShifted{i} = yNorthShifted{i}(downsampledIndices);
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
plot(allX{2},allY{2},'DisplayName','Ground Truth Right Lane(1)','LineWidth',1.2)
hold on
plot(allX{3},allY{3},'DisplayName','Ground Truth Right Lane(2)','LineWidth',1.2)
hold on

plot(myX,myY,'DisplayName','Generated Reference Lane','LineWidth',1.2)
hold on
plot(xEastShifted{1},yNorthShifted{1},'DisplayName','Generated Right Lane(1)','LineWidth',1.2)
hold on
plot(xEastShifted{2},yNorthShifted{2},'DisplayName','Generated Right Lane(2)','LineWidth',1.2)

% plot(allX_OSM,allY_OSM,'DisplayName','OSM Road','LineWidth',1.2 )
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
for i = 1:numel(segments{1})
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

%Computation time
compTimeApprox = [];
for i = 1:numel(segments)
    tempTime = 0;
    for j = 1:numel(segments{i})
        tempTime = tempTime + segments{i}(j).computationTime;
    end
    
    compTimeApprox = [compTimeApprox tempTime];
end
compTimeApprox

%%
numAllClothoid = numel(all_clothoids{1});
disp(['The road initially had ', num2str(numAllClothoid) ,' clothoids.'])
disp(['After approximation and combination of segments the road has ', num2str(numFinalClothoids),...
    ' clothoids and ', num2str(numArcs) , ' arcs were used.'])
disp(['Additionaly, ',num2str(numLines), ' lines were used.'])
disp(['Initially there were ', num2str(numAllClothoid) ' segments. After combination there are ', num2str(numFinalClothoids + numLines), ' segments.'])
save('last_work_V9')
