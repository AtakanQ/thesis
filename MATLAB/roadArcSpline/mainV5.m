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

num_arcs = length(curvature);
arcSegments = cell(1,num_arcs);
radii = abs(1./curvature);
figure;
for i=1:(num_arcs-1)
    init_pos =[xEast(i + 1) yNorth(i + 1)];
    next_pos = [xEast(i + 2) yNorth(i + 2)];
    next_next_pos = [xEast(i + 3) yNorth(i + 3)];

    [~, init_tan] = findTangent(init_pos,next_pos,centers(i,:));
    [~, final_tan] = findTangent(next_pos,next_next_pos,centers(i+1,:));

    if(curvature(i) > 0 ) % turn left
        init_tan = init_tan - pi/2;
        init_tan = mod( rad2deg(init_tan),360 );
        final_tan = final_tan - pi/2;
        final_tan = mod( rad2deg(final_tan),360 );
    else%turn right
        init_tan = init_tan + pi/2;
        init_tan = mod( rad2deg(init_tan),360 );
        final_tan = final_tan + pi/2;
        final_tan = mod( rad2deg(final_tan),360 );
    end

    arcSegments{i} = arcSegment_v3(centers(i,:), init_tan, ...
        final_tan, radii(i),num_arc_points);
    hold on
    arcSegments{i}.plotArc();
    axis equal
end



% figure;
% plot(xEast,yNorth)
% title('Real road')
% hold on
% for m = 2:(length(k)-1)
%     radius = abs(1/k(m));
%     startAngle = mod(rad2deg( -sign( k(m)   ) * pi/2 + theta(m)) , 360);
% 
%     endAngle   = mod( rad2deg( -sign( k(m) ) * pi/2 + theta(m+1)), 360 );
% 
%     rad2deg(theta(m))
%     % arcSegment_v2()
%     myArc = arcSegment_v3(centers(m,:), startAngle, endAngle, radius,num_points);
%     myArc.plotArc();
%     hold on
%     plot(centers(m-1,1),centers(m-1,2),'*','MarkerSize',15)
%     hold on
%     axis equal
% end    







