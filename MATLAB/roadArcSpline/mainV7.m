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

%Use left most lane
for j = 1:length(laneCenters)
    xEastCenter{j} = laneCenters(j).xEast;
    yNorthCenter{j} = laneCenters(j).yNorth;
end


% figure;
% for i = 1:size(laneCenters,2)    
%     plot(laneCenters(i).xEast,laneCenters(i).yNorth,'DisplayName',strcat('Lane ',num2str(i)))
%     hold on
%     axis equal
% end
% plot(xEast,yNorth,'DisplayName','OSM')
% legend()

% if(folderName == 'autobahn_4') % there is some bug with this road
%     xEast = xEast(15:end);
%     yNorth = yNorth(15:end);
% end

% Use clothoid fitting
for i = 1:length(xEastCenter)
    [theta_GT{i},curvature_GT{i},dk{i},L{i},...
        nevalG1,nevalF,iter,Fvalue,Fgradnorm] = ...
        G1spline( [xEastCenter{i} yNorthCenter{i}]);

    [all_clothoids{i}] = ...
        generateClothoids(xEastCenter{i},yNorthCenter{i},theta_GT{i},curvature_GT{i},dk{i},L{i});
end

%% Represent the road
lineCfg.lineDegreeDeviation = 0.2; % Allowed heading devation at the end of the segment (degrees)
lineCfg.rmsThreshold = 0.1; % RMS deviation from real road (meters)
% disp('This implementation has no lines!!!!!!!')
% lineCfg.rmsThreshold = -0.1; 
lineCfg.maximumAllowedDistance = 0.15; % Maximum deviation from real road (meters)
lineCfg.numberOfPoints = 500; % Number of datapoints along the line segment

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
%% Test
figure;
for i = 2:(length(all_clothoids{2})-1)
    plot(all_clothoids{2}(i).allX,all_clothoids{2}(i).allY)
    hold on
    plot(segments{2}(i-1).allX,segments{2}(i-1).allY)
    axis equal
end

%% Generate other lanes
laneWidth = 3.6; % meters

[otherLanes,xEastShifted,yNorthShifted] = generateOtherLanes(segments{3},laneWidth,2,all_clothoids);
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
        if j == 3
            if(segments{3}(i-1).type == "clothoid")
                myX = [myX; segments{3}(i-1).allX'];
                myY = [myY; segments{3}(i-1).allY'];
            elseif(segments{3}(i-1).type == "line")
                myX = [myX; segments{3}(i-1).allX];
                myY = [myY; segments{3}(i-1).allY]; 
            end
        end
    end
end
%% Plot the results

desiredNumElements = 10000;  % Replace with the desired number
for i = 1:numel(allX)
    downsamplingFactor = floor(numel(allX{i}) / desiredNumElements);
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
axis equal
legend()