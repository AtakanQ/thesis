%% REAL DATA 
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


%generate clothoid between consecutive points. 
num_clothoids = length(curvature) - 1;
dummy_arcSeg= arcSegment;

figure;
for i=1:num_clothoids
    init_pos = [xEast(i) yNorth(i)]; 
    init_tan = theta(i); 
    all_clothoids(i) = clothoid(init_pos,init_tan,k(i),k(i)+dk(i)*L(i),...
        L(i),20,dummy_arcSeg);

    all_clothoids(i).generateArcSegments();
    all_clothoids(i).plotPlain();
    hold on
end
title('Clothoid Path')
xlabel('x (m)')
ylabel('y (m)')
grid on