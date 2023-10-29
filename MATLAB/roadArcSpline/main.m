close all
clear
%% Define points

pts = [5 5; 25 30; 55 55; 110 100;120 115; 125 130; 135 160];

myRadii = findCurvature(pts);
figure;
plot(pts(:,1), pts(:,2))
hold on
plot(pts(:,1), pts(:,2),'*')
title('Plot of Hand Given Data Points')

%% Define triangle

trng_pts = [50 10; 30 20; 40 30];

myRadii_trng = findCurvature(trng_pts);
figure;
plot(trng_pts(:,1), trng_pts(:,2))
hold on
plot(trng_pts(:,1), trng_pts(:,2),'*')
title('Plot of Hand Given Data Points')

%% Define circular points

theta = 0:0.01:2*pi;
radius = 600;
x = radius*cos(theta);
y = radius*sin(theta);


myRadii = findCurvature([x ;y]');
figure;
plot(x, y)
hold on
plot(x, y,'*')
title("Generated Sample Circle")

disp("Radius of the sample circle:")
disp(num2str(1/myRadii(5), 100));

% for i = 1:length(myRadii)
%     disp(num2str(1/myRadii(i), 100));
% end

%% Define Beziér Curve
close all
clear
clc
% Control points:
P0 = [0, 0];
P1 = [40 60];
P2 = [100 70];
P3 = [150 80];
P4 = [300 200];
P5 = [50 100];

% x,y coordinates and curvature
[x,y,tangent,curvature] = generateBezier(P0,P1,P2,P3,P4,P5);


% x = x(1:5:length(x));
% y = y(1:5:length(y));
% curvature = curvature(1:5:length(curvature));

[bezierCurvature, centers] = findCurvature([x' y']);


figure;
plot(x,y, 'LineWidth', 1,'Color',[1 0 0])
title("Generated Sample Bezier Curve")
axis equal
% plotMVRCcircles(x,y,centers,bezierCurvature,1/0.0001);


figure;
t = 0:0.001:1;
plot(t,curvature)
title("Generated Bezier Curve's Curvature (Analytical)")
xlabel('t')
ylabel('Curvature')


figure;
plot(linspace(0,1,length(bezierCurvature)),bezierCurvature)
title("Generated Bezier Curve's Curvature (MVRC method)")
xlabel('t')
ylabel('Curvature')


% Find the angle differences 
%########### USELESS, OBSOLETE###########3
% [start_angles, end_angles] = findStartEndAngles(x ,y);
allPoints = [];
figure;
for k = 1:floor((length(centers))/2)
    start_coor = [x(2*k-1) y(2*k-1)];
    end_coor = [x(2*k+1) y(2*k+1)];
    diff_start = start_coor - centers(2*k,:);
    diff_end = end_coor - centers(2*k,:);

    start_angle = atan2(diff_start(2), diff_start(1));
    end_angle = atan2(diff_end(2), diff_end(1));
    start_angle(start_angle < 0) = start_angle(start_angle < 0) + 2*pi;
    end_angle(end_angle < 0) = end_angle(end_angle < 0) + 2*pi;

    temp = arcSegment(centers(2*k,:), rad2deg(start_angle), rad2deg(end_angle),abs(1/bezierCurvature(2*k)) );

    hold on
    temp.plotArc();
    allPoints = [allPoints; temp.Coordinates];
end
title("Bezier Curve Approximated with Arc Spline")
xlabel('x coordinate')
ylabel('y coordinate')
axis equal

desiredLength = length(x);
downsamplingFactor = ceil(length(allPoints) / desiredLength);
allPointsDownsampled = allPoints(1:downsamplingFactor:end,:);

% Calculate the difference between every point. Note that downsampled
% points array will not be equal to the number of real analytical (x,y)
% pairs. Therefore computing the difference between every downsampled point
% and corresponding analytical point should be enough for distance
% computation.
distances = zeros(length(allPointsDownsampled),1);

for i = 1:length(allPointsDownsampled)
    distances(i) = norm( allPointsDownsampled(i,:) - [x(i) y(i)] );
end
figure;
plot(distances,'.')
title("Distance Between Analytical and Arc Approximation")
xlabel('Coordinate index')
ylabel('Distance')

%Compute the tangent angle of analytical and generated curves
analytical_angles = atan2(tangent(:,2),tangent(:,1));
% Plot the tangents to see the results.
% figure;
% quiver(x,y,tangent(:,1)',tangent(:,2)')
% title(" Tangent Angle of Every Point of Analytical Calculation")
% xlabel('x')
% ylabel('y')



% Compute the tangents. We have the arc spline so 
% perhaps there is a better way to do this...

tangent_arc_spline = diff(allPointsDownsampled);
figure;
quiver(allPointsDownsampled(1:end-1,1),allPointsDownsampled(1:end-1,2),tangent_arc_spline(:,1),tangent_arc_spline(:,2))
title(" Tangent Angle of Every Point of MVRC Method")
xlabel('x')
ylabel('y')
MVRC_angles = atan2(tangent_arc_spline(:,2),tangent_arc_spline(:,1));

figure;
plot(analytical_angles,'.');
hold on
plot(MVRC_angles,'.');
title('Angle Values of Tangents of Analytical and MVRC Methods')
xlabel('Coordinate index')
ylabel('Angle Value (radians)')
legend('Analytical','MVRC')

%Calculate the difference between points
% xy = [diff(x') diff(y')];
% lengths = sqrt(sum(xy.^2,2));
% figure;
% plot(t(1:end-1),lengths)
% title('Lengths along the curve')
% xlabel('t')
% ylabel('Lengths')

%% Real Road
close all
clear
load('../sharp.mat')

x = data.RoadSpecifications(1, 1).Centers(:,1);
y = data.RoadSpecifications(1, 1).Centers(:,2);
path_len = sum(norm(diff([x y])));
sprintf('Approximate real road path length: %d m',path_len)
figure;
% plot3(x,y,1:length(x))
plot(x,y,'LineWidth',2)
title('Real Road')
xlabel('m')
ylabel('m')
axis equal
grid on

%From plot get datapoints. For sharp turn only pick [5 22]

picked_x = x(5:22);
picked_y = y(5:22);
hold on
plot(picked_x,picked_y,'Color',[1 0 0],'LineWidth',2)
legend('Full Road','Picked Road Segment')

[curvature, centers] = findCurvature([picked_x picked_y]);

figure;
plot(curvature*1000,'*','MarkerSize',6)
title('Curvature')
xlabel('Data index')
ylabel('(Radius^-^1)*1000')
axis equal
grid on

plotCircles(curvature,centers,picked_x,picked_y)
hold on
plot(picked_x,picked_y,'.','Color',[1 0 0])
legend('Estimation','Picked Road Segment')