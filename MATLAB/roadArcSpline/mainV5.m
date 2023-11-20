%% REAL DATA 
%inherited from V3
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

%% Test new methods
% Every clothoid has 420 = ( (clothoid_order + 1) * 20) data points in them.

% arcSegment_v3(center, startAngle, endAngle, radius,num_points)
% turn left positive curvature

% fitCircles(curvature,centers,theta)
num_arc_points = 420;


[arcSegments] = plotCircles_v2(curvature,centers,xEast,yNorth,num_arc_points);
hold on
plot(xEast,yNorth,'*','Color',[1 0 0])
legend('Arcs','Road data points');


numSegments = length(arcSegments);
diffx = cell(numSegments,1);
diffy = cell(numSegments,1);

for j = 1:numSegments
    diffx{j} = arcSegments{j}.Coordinates(:,1) - all_clothoids(j).allX';
    diffy{j} = arcSegments{j}.Coordinates(:,2) - all_clothoids(j).allY';
end




