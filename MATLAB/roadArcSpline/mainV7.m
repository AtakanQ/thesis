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
HEREname = 'A4_laneData_v3.mat';

[xEast, yNorth,number_of_roads,refLat,refLon] = ...
    retrieveOSM_v2(lat1, lat2, lon1, lon2, roadName,folderName);

[laneBorders,laneCenters] = retrieveHERE_v2(folderName,HEREname,refLat,refLon,[lat1 lat2],[lon1 lon2]);

%Use left most lane
for j = 1:length(laneCenters)
    xEastCenter{j} = laneCenters(j).xEast;
    yNorthCenter{j} = laneCenters(j).yNorth;
end

for j = 1:length(laneBorders)
    xEastCenter_LB{j} = laneBorders(j).xEast;
    yNorthCenter_LB{j} = laneBorders(j).yNorth;
end
% figure;
% for i = 1:size(laneCenters,2)    
%     plot(laneCenters(i).xEast,laneCenters(i).yNorth,'DisplayName',strcat('Lane ',num2str(i)))
%     hold on
%     axis equal
% end
% plot(xEast,yNorth,'DisplayName','OSM')
% legend()

if(folderName == 'autobahn_4') % there is some bug with this road
    xEast = xEast(15:end);
    yNorth = yNorth(15:end);
end

% Use clothoid fitting
for i = 1:length(xEastCenter)
    [theta_GT{i},curvature_GT{i},dk{i},L{i},...
        nevalG1,~,~,~,~] = ...
        G1spline( [xEastCenter{i} yNorthCenter{i}]);

    [all_clothoids{i}] = ...
        generateClothoids(xEastCenter{i},yNorthCenter{i},theta_GT{i},curvature_GT{i},dk{i},L{i});
end

for i = 1:length(xEastCenter_LB)
    [theta_GT_LB{i},curvature_GT_LB{i},dk_LB{i},L_LB{i},...
        nevalG1_LB,~,~,~,~] = ...
        G1spline( [xEastCenter_LB{i} yNorthCenter_LB{i}]);

    [all_clothoids_LB{i}] = ...
        generateClothoids(xEastCenter_LB{i},yNorthCenter_LB{i},theta_GT_LB{i},curvature_GT_LB{i},dk_LB{i},L_LB{i});
end


numAllClothoid = numel(all_clothoids{3});

[theta_OSM,curvature_OSM,dk_OSM,L_OSM,...
    ~,~,~,~,~] = ...
    G1spline( [xEast yNorth]);
all_clothoids_OSM = generateClothoids(xEast,yNorth,...
    theta_OSM,curvature_OSM,dk_OSM,L_OSM);

%% Represent the road
lineCfg.lineDegreeDeviation = 0.2; % Allowed heading devation at the end of the segment (degrees)
lineCfg.rmsThreshold = 0.01; % RMS deviation from real road (meters)
% disp('This implementation has no lines!!!!!!!')
% lineCfg.rmsThreshold = -0.1; 
lineCfg.maximumAllowedDistance = 0.15; % Maximum deviation from real road (meters)
% lineCfg.numberOfPoints = 500; % Number of datapoints along the line
% segment OBSOLETE

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
%     combineSegments(segments{3},all_clothoids{3}(2:end-1),errorCfg);
% 
% segments{3} = mergedSegments;
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
laneWidth = 3.6; % meters
numLanes = 2;
[otherLanes,xEastShifted,yNorthShifted] = generateOtherLanes(segments{3},laneWidth,numLanes);
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
for j = 1:length(segments{3})
    if(segments{3}(j).type == "clothoid")
        numFinalClothoids = numFinalClothoids + 1;
        numArcs = numArcs + segments{3}(j).numArcs + 1;
        myX = [myX; segments{3}(j).allX'];
        myY = [myY; segments{3}(j).allY'];
    elseif(segments{3}(j).type == "line")
        numLines = numLines + 1;
        myX = [myX; segments{3}(j).allX];
        myY = [myY; segments{3}(j).allY]; 
    end
end

%% Plot the results

desiredNumElements = 10000;  % Replace with the desired number
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

allX_OSM = [];
allY_OSM = [];
for i = 1:numel(all_clothoids_OSM)
    allX_OSM = [allX_OSM; all_clothoids_OSM(i).allX'];
    allY_OSM = [allY_OSM; all_clothoids_OSM(i).allY'];
end

downsamplingFactor = floor(numel(allX_OSM) / desiredNumElements);
downsampledIndices = 1:downsamplingFactor:numel(allX_OSM);
allX_OSM = allX_OSM(downsampledIndices);
allY_OSM = allY_OSM(downsampledIndices);

figure
plot(allX{3},allY{3},'DisplayName','Ground Truth Reference','LineWidth',1.2)
hold on
plot(allX{2},allY{2},'DisplayName','Ground Truth Right Lane(1)','LineWidth',1.2)
hold on
plot(allX{1},allY{1},'DisplayName','Ground Truth Right Lane(2)','LineWidth',1.2)
hold on

plot(myX,myY,'DisplayName','Generated Reference Lane','LineWidth',1.2)
hold on
plot(xEastShifted{1},yNorthShifted{1},'DisplayName','Generated Right Lane(1)','LineWidth',1.2)
hold on
plot(xEastShifted{2},yNorthShifted{2},'DisplayName','Generated Right Lane(2)','LineWidth',1.2)

plot(allX_OSM,allY_OSM,'DisplayName','OSM Road','LineWidth',1.2 )

axis equal
legend()



%% Compute rms and curvatures
segmentLength = 100;
rms_array = cell(length(xEastShifted),1);
max_err_array = cell(length(xEastShifted),1);
curvatures = cell(length(xEastShifted),1);
roadLengths = cell(length(xEastShifted),1);
for i = 1:length(xEastShifted)
    if i == 2
        idx = 1;
    elseif i == 1
        idx = 2;
    end

    rms_array{i} = zeros(floor(length(xEastShifted{i})/segmentLength),1);
    max_err_array{i} = zeros(floor(length(xEastShifted{i})/segmentLength),1);
    roadLengths{i} = zeros(floor(length(xEastShifted{i})/segmentLength),1);
    ground_truth_xy = [allX{idx} allY{idx}];
    for j = 1:floor(length(xEastShifted{i})/segmentLength)
        start_idx = (j-1) * segmentLength + 1;
        end_idx = j*segmentLength - 1;
        measurement_xy = [xEastShifted{i}(start_idx:end_idx) yNorthShifted{i}(start_idx:end_idx)];
        
        
        roadLengths{i}(j) = calculateRoadLength(measurement_xy);

        % [rms_error, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy);
        [rms_array{i}(j), max_err_array{i}(j), errors] = ...
            computeSegmentError(measurement_xy,ground_truth_xy);
    end

    %Compute curvatures from ground truth data
    curvatures{i} = findCurvature(ground_truth_xy);
    
    curvatures{i} = medfilt1(curvatures{i}, 10);

    

    figure;
    plot(rms_array{i},'DisplayName', strcat('RMS Error:',num2str(i)))
    hold on
    plot(max_err_array{i},'DisplayName', strcat('Max Error:',num2str(i)))
    ylabel('Error (m)')
    yyaxis right
    ylabel('Curvature (m^-^1)')
    xlabel('Segment Number')
    xAxis = linspace(1,length(rms_array{i}),length(curvatures{i}));
    plot(xAxis,curvatures{i} ,'DisplayName', strcat('Curvature:',num2str(i)))
    legend()
    title('Error With Respect To HERE Map')

    figure;
    plot(rms_array{i},'DisplayName', strcat('RMS Error:',num2str(i)))
    hold on
    plot(max_err_array{i},'DisplayName', strcat('Max Error:',num2str(i)))
    ylabel('Error (m)')

    hold on
    yyaxis right
    plot(roadLengths{i},'DisplayName', ['Path Length: ',num2str(i)])
    ylabel('Road Length (m)')
    xlabel('Segment Number')
    legend()
    title('Error With Respect To HERE Map')


end

%% TEST
% % ground_truth_xy = [allX{2} allY{2}];
% % j = 27;
% % start_idx = (j-1) * segmentLength + 1;
% % end_idx = j*segmentLength - 1;
% % measurement_xy = [xEastShifted{1}(start_idx:end_idx) yNorthShifted{1}(start_idx:end_idx)];
% % figure;
% % plot(measurement_xy(:,1),measurement_xy(:,2),'Color',[0 1 0]);
% % hold on
% % plot(ground_truth_xy(:,1),ground_truth_xy(:,2),'Color',[1 0 0]);
% % measurement_xy(1,1)
% % measurement_xy(1,2)
% % axis equal

% [~, ~, to_left] = computeSegmentError([allX{2} allY{2}],[allX{1} allY{1}]);
% [~, ~, to_right] = computeSegmentError([allX{2} allY{2}],[allX{3} allY{3}]);
% figure;
% plot(to_left)
% title('Distance Between Middle and Left Lane')
% xlabel('Index')
% ylabel('Distance (m)')
% figure;
% plot(to_right)
% title('Distance Between Middle and Right Lane')
% xlabel('Index')
% ylabel('Distance (m)')

%% OSM error
% rms_OSM = zeros(floor(length(xEastShifted{1})/segmentLength),1);
% max_err_OSM = zeros(floor(length(xEastShifted{1})/segmentLength),1);
% 
% ground_truth_xy = [allX_OSM allY_OSM];
% for j = 1:floor(length(xEastShifted{1})/segmentLength)
%     start_idx = (j-1) * segmentLength + 1;
%     end_idx = j*segmentLength - 1;
% 
%     %middle lane
%     measurement_xy = [xEastShifted{1}(start_idx:end_idx) yNorthShifted{1}(start_idx:end_idx)];
% 
%     % [rms_error, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy);
%     [rms_OSM(j), max_err_OSM(j), errors] = ...
%         computeSegmentError(measurement_xy,ground_truth_xy);
% end
% curvature_OSM = findCurvature(ground_truth_xy);
% curvatures{i} = medfilt1(curvatures{i}, 10);
% figure;
% plot(rms_OSM,'DisplayName', strcat('RMS Error:',num2str(1)))
% hold on
% plot(max_err_OSM,'DisplayName', strcat('Max Error:',num2str(1)))
% ylabel('Error (m)')
% yyaxis right
% ylabel('Curvature (m^-^1)')
% xlabel('Segment Number')
% xAxis = linspace(1,length(rms_OSM),length(curvature_OSM));
% plot(xAxis,curvature_OSM ,'DisplayName', strcat('Curvature:',num2str(1)))
% legend()
% title('Error With Respect o OSM Map')

%%
disp(['The road initially had ', num2str(numAllClothoid) ,' clothoids.'])
disp(['After approximation and combination of segments the road has ', num2str(numFinalClothoids),...
    ' clothoids and ', num2str(numArcs) , ' arcs were used.'])
disp(['Additionaly, ',num2str(numLines), ' lines were used.'])
disp(['Initially there were ', num2str(numAllClothoid) ' segments. After combination there are ', num2str(numFinalClothoids + numLines), ' segments.'])
save('last_work_V7')

