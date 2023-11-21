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
num_clothoids = length(curvature);
dummy_arcSeg= arcSegment;

figure;
for i=1:num_clothoids
    init_pos = [xEast(i) yNorth(i)]; 
    init_tan = theta(i); 

    %OBSOLETE WITH clothoid_v2
    % all_clothoids(i) = clothoid(init_pos,init_tan,k(i),k(i)+dk(i)*L(i),...
    %     L(i),20,dummy_arcSeg);

    % Using k(i+1) instead of k(i)+dk(i)*L(i) should be better ?. TODO
    all_clothoids(i) = clothoid_v2(init_pos, init_tan, ...
        k(i), k(i)+dk(i)*L(i),L(i),5000 );
    
    %OBSOLETE WITH clothoid_v2
    % all_clothoids(i).generateArcSegments();

    all_clothoids(i).plotPlain();
    hold on

    % v2_cloth = clothoid_v2(init_pos, init_tan,k(i), k(i)+dk(i)*L(i),L(i),1000 );
    % figure;
    % v2_cloth.plotPlain();
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
num_arc_points = 500;


[arcSegments] = plotCircles_v2(curvature,centers,xEast,yNorth,num_arc_points);
hold on
plot(xEast,yNorth,'*','Color',[1 0 0])
% legend('Arcs','Road data points');


errors = computeError(arcSegments,all_clothoids);

figure;
plot(errors{1})
title('Error')
figure;
plot(errors{10})
title('Error')
figure;
plot(errors{21})
title('Error')
figure;
plot(errors{31})
title('Error')