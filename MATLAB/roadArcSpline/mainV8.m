set(groot, 'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on', 'defaultAxesZGrid', 'on');
%% REAL DATA 
close all
clear
addpath('../../CLOTHOIDFITTING/G1fitting')

lat1 = 51.495379;
lat2 = 51.444991;
lon1 = 10.658041;
lon2 = 10.807802;
folderName = 'autobahn_38';
roadName = 'A 38'; 
HEREname = 'A38_laneData_v2.mat';

[xEast, yNorth,number_of_roads,refLat,refLon] = ...
    retrieveOSM_v2(lat1, lat2, lon1, lon2, roadName,folderName);

[OSM_lats,OSM_lons,h] = enu2geodetic(xEast,yNorth,0,refLat,refLon,0,wgs84Ellipsoid);
set(groot, 'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on', 'defaultAxesZGrid', 'on');
% figure;
% geoplot(OSM_lats,OSM_lons,'LineWidth',2)
% title("OpenStreetMap Data Example","FontSize",12)
% geobasemap satellite

[laneBorders,laneCenters] = retrieveHERE_v2(folderName,HEREname,refLat,refLon,[lat1 lat2], [lon1 lon2]);

%Use left most lane
laneCntr = 1;
for j = 3:length(laneCenters) %DONT TAKE FIRST
    xEastCenter{laneCntr} = laneCenters(j).xEast;
    yNorthCenter{laneCntr} = laneCenters(j).yNorth;
    laneCntr = laneCntr + 1;
end

for j = 1:length(laneBorders)
    xEastCenter_LB{j} = laneBorders(j).xEast;
    yNorthCenter_LB{j} = laneBorders(j).yNorth;
end

figure;

for i = 1:size(xEastCenter,2)    
    plot(xEastCenter{i},yNorthCenter{i},'DisplayName',strcat('Lane ',num2str(i)))
    hold on
    axis equal
end
title("Ground Truth Road")
grid on
% plot(xEast,yNorth,'DisplayName','OSM')
legend()

if(strcmp(folderName , "autobahn_4")) % there is some bug with this road
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

figure;
for i = 1:numel(all_clothoids{1})
    [HERE_lats,HERE_lons,h] = enu2geodetic(all_clothoids{1}(i).allX,all_clothoids{1}(i).allY...
        ,0,refLat,refLon,0,wgs84Ellipsoid);

    geoplot(HERE_lats,HERE_lons,'LineWidth',2,'Color',[0.3010, 0.7450, 0.9330])
    hold on
    title("G1 Clothoid Fitted Road (Autobahn 38)","FontSize",13)
    geobasemap satellite
end



% Use clothoid fitting to fit lane boundaries
for i = 2:length(xEastCenter_LB)
    [theta_GT_LB{i},curvature_GT_LB{i},dk_LB{i},L_LB{i},...
        nevalG1_LB,~,~,~,~] = ...
        G1spline( [xEastCenter_LB{i} yNorthCenter_LB{i}]);

    [all_clothoids_LB{i}] = ...
        generateClothoids(xEastCenter_LB{i},yNorthCenter_LB{i},theta_GT_LB{i},curvature_GT_LB{i},dk_LB{i},L_LB{i});
end

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
save("roadSegmentData.mat","roadSegmentData")
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
% 
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
laneWidth = 3.8; % meters
numLanes = 1; % other lanes
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

desiredNumElements = 10000;  % Replace with the desired number
desiredNumElements_GT = 50000;
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
    allX_OSM = [allX_OSM all_clothoids_OSM(i).allX];
    allY_OSM = [allY_OSM all_clothoids_OSM(i).allY];
end

% downsamplingFactor = floor(numel(allX_OSM) / desiredNumElements);
% downsampledIndices = 1:downsamplingFactor:numel(allX_OSM);
% allX_OSM = allX_OSM(downsampledIndices);
% allY_OSM = allY_OSM(downsampledIndices);

figure
plot(allX{1},allY{1},'DisplayName','Ground Truth Reference','LineWidth',1.2)
hold on
plot(allX{2},allY{2},'DisplayName','Ground Truth Right Lane(1)','LineWidth',1.2)
hold on
% plot(allX{1},allY{1},'DisplayName','Ground Truth Right Lane(2)','LineWidth',1.2)
% hold on

plot(myX,myY,'DisplayName','Generated Reference Lane','LineWidth',1.2)
hold on
plot(xEastShifted{1},yNorthShifted{1},'DisplayName','Generated Right Lane(1)','LineWidth',1.2)
% hold on
% plot(xEastShifted{2},yNorthShifted{2},'DisplayName','Generated Right Lane(2)','LineWidth',1.2)

% plot(allX_OSM,allY_OSM,'DisplayName','OSM Road','LineWidth',1.2 )
title("Real Road and Generated Road")
axis equal
xlabel("xEast (m)")
ylabel("yNorth (m)")
grid on
legend()


%% Compute rms and max error of a picked shifted lane (arc spline)
segmentNumShifted = 282;
segmentLengthShifted = 1000;

xEastShifted = otherLanes{1}(segmentNumShifted).allX';
yNorthShifted = otherLanes{1}(segmentNumShifted).allY';
ground_truth_xy_shifted = [ [all_clothoids{2}(segmentNumShifted+1).allX'; all_clothoids{2}(segmentNumShifted+2).allX']...
    [all_clothoids{2}(segmentNumShifted+1).allY'; all_clothoids{2}(segmentNumShifted+2).allY']];

rms_array_shifted = zeros(floor(length(xEastShifted)/segmentLengthShifted),1);
max_err_array_shifted = zeros(floor(length(yNorthShifted)/segmentLengthShifted),1);

for j = 1:numel(rms_array_shifted)
    start_idx = (j-1) * segmentLengthShifted + 1;
    end_idx = j*segmentLengthShifted - 1;
    measurement_xy = [xEastShifted(start_idx:end_idx) yNorthShifted(start_idx:end_idx)];

    % [rms_error, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy);
  [rms_array_shifted(j), max_err_array_shifted(j), errors] = ...
        computeSegmentError(measurement_xy,ground_truth_xy_shifted);
end
% figure;
% plot(xEastShifted,yNorthShifted,'DisplayName',"Meas")
% hold on
% plot(ground_truth_xy_shifted(:,1),ground_truth_xy_shifted(:,2),'DisplayName',"GT")
% axis equal
% legend()

figure;
plot(rms_array_shifted,'DisplayName', 'RMS Error','LineWidth',1.5)
hold on
plot(max_err_array_shifted,'DisplayName', 'Max Error','LineWidth',1.5)
ylabel('Error (m)','FontSize',13)
xlabel('Index of 10 m subsegment','FontSize',13)
legend()
title('Error Between Parallel Shifted Lane and Ground Truth','FontSize',13)
ylim([0 0.3])
xlim([1 numel(rms_array_shifted)])

%% Compute rms and curvatures of a picked shifted lane (line segment)
segmentNumShifted = 272;
segmentLengthShiftedLine = 1000;

xEastShiftedLine = otherLanes{1}(segmentNumShifted).allX;
yNorthShiftedLine = otherLanes{1}(segmentNumShifted).allY;
ground_truth_xy_shifted_line = [ [all_clothoids{2}(segmentNumShifted+1).allX'; all_clothoids{2}(segmentNumShifted+2).allX']...
    [all_clothoids{2}(segmentNumShifted+1).allY'; all_clothoids{2}(segmentNumShifted+2).allY']];

rms_array_shifted_line = zeros(floor(length(xEastShiftedLine)/segmentLengthShiftedLine),1);
max_err_array_shifted_line = zeros(floor(length(yNorthShiftedLine)/segmentLengthShiftedLine),1);

for j = 1:numel(rms_array_shifted_line)
    start_idx = (j-1) * segmentLengthShiftedLine + 1;
    end_idx = j*segmentLengthShiftedLine - 1;
    measurement_xy = [xEastShiftedLine(start_idx:end_idx) yNorthShiftedLine(start_idx:end_idx)];

    % [rms_error, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy);
  [rms_array_shifted_line(j), max_err_array_shifted_line(j), ~] = ...
        computeSegmentError(measurement_xy,ground_truth_xy_shifted_line);
end
% figure;
% plot(xEastShifted,yNorthShifted,'DisplayName',"Meas")
% hold on
% plot(ground_truth_xy_shifted(:,1),ground_truth_xy_shifted(:,2),'DisplayName',"GT")
% axis equal
% legend()

figure;
plot(rms_array_shifted_line,'DisplayName', 'RMS Error','LineWidth',1.5)
hold on
plot(max_err_array_shifted_line,'DisplayName', 'Max Error','LineWidth',1.5)
ylabel('Error (m)','FontSize',13)
xlabel('Index of 10 m subsegment','FontSize',13)
legend()
title('Error Between Parallel Shifted Lane and Ground Truth','FontSize',13)
ylim([0 0.15])
xlim([1 numel(rms_array_shifted_line)])

%% Compute rms and curvatures of shifted lanes
% segmentLength = 100;
% rms_array = cell(length(xEastShifted),1);
% max_err_array = cell(length(xEastShifted),1);
% curvatures = cell(length(xEastShifted),1);
% 
% for i = 1:length(xEastShifted)
%     if i == 2
%         idx = 1;
%     elseif i == 1
%         idx = 2;
%     end
% 
%     rms_array{i} = zeros(floor(length(xEastShifted{i})/segmentLength),1);
%     max_err_array{i} = zeros(floor(length(xEastShifted{i})/segmentLength),1);
%     ground_truth_xy = [allX{idx} allY{idx}];
%     for j = 1:floor(length(xEastShifted{i})/segmentLength)
%         start_idx = (j-1) * segmentLength + 1;
%         end_idx = j*segmentLength - 1;
%         measurement_xy = [xEastShifted{i}(start_idx:end_idx) yNorthShifted{i}(start_idx:end_idx)];
% 
%         % [rms_error, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy);
%         [rms_array{i}(j), max_err_array{i}(j), errors] = ...
%             computeSegmentError(measurement_xy,ground_truth_xy);
%     end
% 
%     %Compute curvatures from ground truth data
%     curvatures{i} = findCurvature(ground_truth_xy);
% 
%     curvatures{i} = medfilt1(curvatures{i}, 10);
% 
%     figure;
%     plot(rms_array{i},'DisplayName', 'RMS Error')
%     hold on
%     plot(max_err_array{i},'DisplayName', 'Max Error')
%     ylabel('Error (m)','FontSize',13)
%     % yyaxis right
%     % ylabel('Curvature (m^-^1)')
%     xlabel('Segment Number','FontSize',13)
%     % xAxis = linspace(1,length(rms_array{i}),length(curvatures{i}));
%     % plot(xAxis,curvatures{i} ,'DisplayName', strcat('Curvature:',num2str(i)))
%     legend()
%     title('Euclidian Error Between Shifted Lane and Ground Truth','FontSize',13)
% end
%% Compute rms and max error of a picked reference arcspline segment
segmentNum = 282;
segmentLength = 1000; % samples
xEastRef = [segments{1}(segmentNum).allX];
yNorthRef = [segments{1}(segmentNum).allY];

rms_array_ref = zeros(floor(length(xEastRef)/segmentLength),1);
max_err_array_ref = zeros(floor(length(yNorthRef)/segmentLength),1);
ground_truth_xy = [all_clothoids{1}(segmentNum+1).allX' all_clothoids{1}(segmentNum+1).allY'];

for j = 1:numel(rms_array_ref)
    start_idx = (j-1) * segmentLength + 1;
    end_idx = j*segmentLength - 1;
    measurement_xy = [xEastRef(start_idx:end_idx)' yNorthRef(start_idx:end_idx)'];

    % [rms_error, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy);
  [rms_array_ref(j), max_err_array_ref(j), errors] = ...
        computeSegmentError(measurement_xy,ground_truth_xy);
end
% figure;
% plot(xEastRef,yNorthRef,'DisplayName',"Meas")
% hold on
% plot(ground_truth_xy(:,1),ground_truth_xy(:,2),'DisplayName',"GT")
% axis equal
% legend()

figure;
% plot(rms_array_ref,'DisplayName', 'RMS Error','LineWidth',1.5)
hold on
plot(max_err_array_ref,'DisplayName', 'Max Error','LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980])
ylabel('Error (m)','FontSize',13)
% yyaxis right
% ylabel('Curvature (m^-^1)')
xlabel('Index of 10 m subsegment','FontSize',13)
% xAxis = linspace(1,length(rms_array{i}),length(curvatures{i}));
% plot(xAxis,curvatures{i} ,'DisplayName', strcat('Curvature:',num2str(i)))
legend()
title('Error Between Approximated Base Lane and Ground Truth','FontSize',13)
ylim([0 0.01])
xlim([1 numel(rms_array_ref)])
%% Compute rms and max error of a picked reference line segment
segmentNum = 275;
segmentLength = 1000; % samples
xEastRef = [segments{1}(segmentNum).allX];
yNorthRef = [segments{1}(segmentNum).allY];

rms_array_ref_line = zeros(floor(length(xEastRef)/segmentLength),1);
max_err_array_ref_line = zeros(floor(length(yNorthRef)/segmentLength),1);
ground_truth_xy = [all_clothoids{1}(segmentNum+1).allX' all_clothoids{1}(segmentNum+1).allY'];

for j = 1:numel(rms_array_ref_line)
    start_idx = (j-1) * segmentLength + 1;
    end_idx = j*segmentLength - 1;
    measurement_xy = [xEastRef(start_idx:end_idx) yNorthRef(start_idx:end_idx)];

    % [rms_error, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy);
  [rms_array_ref_line(j), max_err_array_ref_line(j), errors] = ...
        computeSegmentError(measurement_xy,ground_truth_xy);
end
% figure;
% plot(xEastRef,yNorthRef,'DisplayName',"Meas")
% hold on
% plot(ground_truth_xy(:,1),ground_truth_xy(:,2),'DisplayName',"GT")
% axis equal
% legend()

figure;
% plot(rms_array_ref_line,'DisplayName', 'RMS Error','LineWidth',1.5)
hold on
plot(max_err_array_ref_line,'DisplayName', 'Max Error','LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980])
ylabel('Error (m)','FontSize',13)
% yyaxis right
% ylabel('Curvature (m^-^1)')
xlabel('Index of 10 m subsegment','FontSize',13)
% xAxis = linspace(1,length(rms_array{i}),length(curvatures{i}));
% plot(xAxis,curvatures{i} ,'DisplayName', strcat('Curvature:',num2str(i)))
legend()
title('Error Between Approximated Base Lane and Ground Truth','FontSize',13)
% ylim([0 0.01])
xlim([1 numel(rms_array_ref_line)])

%% OSM error
% rms_OSM = zeros(floor(length(xEastShifted{1})/segmentLength),1);
% max_err_OSM = zeros(floor(length(xEastShifted{1})/segmentLength),1);
% 
% ground_truth_xy = [allX_OSM' allY_OSM'];
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
%% Compute distance between centers

[rms_error_GT, max_error_GT, errors_GT] = computeSegmentError([allX{1}(1:10000) allY{1}(1:10000)],[allX{2}(1:10000) allY{2}(1:10000)]);
%% Compute memory usage
roadLen = 0;
numBytes = 0;
for i = 1:numel(segments{1})
    if(segments{1}(i).numArcs>0)
        tempBytes = 16*segments{1}(i).numArcs;
    else
        tempBytes = 16;
    end
    roadLen = roadLen + segments{1}(i).segmentLength;
   numBytes = numBytes + tempBytes;  
end


%%
numAllClothoid = numel(all_clothoids{1});
disp(['The road initially had ', num2str(numAllClothoid) ,' clothoids.'])
disp(['After approximation and combination of segments the road has ', num2str(numFinalClothoids),...
    ' clothoids and ', num2str(numArcs) , ' arcs were used.'])
disp(['Additionaly, ',num2str(numLines), ' lines were used.'])
disp(['Initially there were ', num2str(numAllClothoid) ' segments. After combination there are ', num2str(numFinalClothoids + numLines), ' segments.'])
save('last_work_V8')
