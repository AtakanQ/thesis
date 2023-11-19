%% REAL DATA 
close all
clear

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

%This is just an estimation. Possible errors might arise due to this
%computation.
segment_lengths = sqrt(sum(diffxy.^2,2));

sum_over_curve = cumulativeSum(segment_lengths);
crv_multiplied = curvature*1000;
figure;
plot(sum_over_curve(2:end),crv_multiplied,'*','MarkerSize',6)
title('Curvature')
xlabel('Data index')
ylabel('(Curvature)*1000')
% axis equal
grid on

plotCircles(curvature,centers,xEast,yNorth)
hold on
plot(xEast,yNorth,'.','Color',[1 0 0])
% hold on
% plot(xEast,yNorth,'Color',[0 1 0])
axis equal

%generate clothoid between consecutive points. 
num_clothoids = length(curvature) - 1;
dummy_arcSeg= arcSegment;

figure;
for i=1:num_clothoids
    % clothoid(init_pos,init_tan, init_curvature, final_curvature,...
               % length,order,arcSegClass)

    init_pos =[xEast(i + 1) yNorth(i + 1)];
    next_pos = [xEast(i + 2) yNorth(i + 2)];
    next_next_pos = [xEast(i + 3) yNorth(i + 3)];

    % vector = next_pos - init_pos;
    % init_tan = atan2(vector(2),vector(1));
    [~, init_tan] = findTangent(init_pos,next_pos,centers(i,:));
    [~, final_tan] = findTangent(next_pos,next_next_pos,centers(i+1,:));

    if(curvature(i)*curvature(i+1) > 0) % they are the same sign.
        segLen = findSegmentLen(init_tan,curvature(i),final_tan,curvature(i+1));
    elseif(curvature(i)*curvature(i+1) < 0) % opposite sign.
        segLen = segment_lengths(i+1);
    end
    all_clothoids(i) = clothoid(init_pos,init_tan,curvature(i),curvature(i+1),...
        segLen,20,dummy_arcSeg);

    all_clothoids(i).generateArcSegments();
    all_clothoids(i).plotPlain();
    hold on
end
title('Clothoid Path')
xlabel('x (m)')
ylabel('y (m)')
grid on