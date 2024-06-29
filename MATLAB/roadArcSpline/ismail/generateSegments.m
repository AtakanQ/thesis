load('peacthtree_latlon.mat')
addpath('../../../CLOTHOIDFITTING/G1fitting')
addpath('..')


refLat = mean(peachtree_latlon(:,1));
refLon = mean(peachtree_latlon(:,2));

[xEastCenter , yNorthCenter , ~] = geodetic2enu(peachtree_latlon(:,1), ...
    peachtree_latlon (:,2), 0, refLat, refLon, 0, wgs84Ellipsoid);

xEastCenter(15) = [];
yNorthCenter(15) = [];
xEastCenter(21) = [];
yNorthCenter(21) = [];

figure;
plot(xEastCenter, yNorthCenter)
axis equal
[theta_GT,curvature_GT,dk,L,...
    nevalG1,~,~,~,~] = ...
    G1spline( [xEastCenter yNorthCenter]);

[all_clothoids] = ...
    generateClothoids(xEastCenter,yNorthCenter,theta_GT,curvature_GT,dk,L);

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

segments = ...
    representRoad_v2(xEastCenter(2:end-1),yNorthCenter(2:end-1),...
    theta_GT(2:end-1),curvature_GT(2:end),...
    L(2:end-1),all_clothoids(2:end-1),lineCfg,arcCfg,DEBUG);

