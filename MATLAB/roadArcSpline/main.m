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
P1 = [10, 15];
P2 = [15, 25];
P3 = [25, 20];
P4 = [40, 10];
P5 = [30, 80];

% x,y coordinates and curvature
[x,y,curvature] = generateBezier(P0,P1,P2,P3,P4,P5);


x = x(1:5:length(x));
y = y(1:5:length(y));
curvature = curvature(1:5:length(curvature));

[bezierCurvature, centers] = findCurvature([x' y']);


figure;
plot(x,y, 'LineWidth', 1,'Color',[1 0 0])
title("Generated Sample Bezier Curve")
axis equal
% plotMVRCcircles(x,y,centers,bezierCurvature,1/0.0001);


figure;
t = 0:0.001:1;
plot(t(1:5:end),curvature)
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

figure;
for k = 1:((length(centers)-1)/2)
    start_coor = [x(2*k-1) y(2*k-1)];
    end_coor = [x(2*k+1) y(2*k+1)];
    diff_start = start_coor - centers(2*k,:);
    diff_end = end_coor - centers(2*k,:);

    start_angle = atan2(diff_start(2), diff_start(1));
    end_angle = atan2(diff_end(2), diff_end(1));
    start_angle(start_angle < 0) = start_angle(start_angle < 0) + 2*pi;
    end_angle(end_angle < 0) = end_angle(end_angle < 0) + 2*pi;

    % disp('Angles')
    % rad2deg(start_angle)
    % rad2deg(end_angle)
    % 1/bezierCurvature(k)
    % centers(k,:)
    temp = arcSegment(centers(2*k,:), rad2deg(start_angle), rad2deg(end_angle),abs(1/bezierCurvature(2*k)) );
    hold on
    temp.plotArc();

    % plot([start_coor(1) end_coor(1)],[start_coor(2) end_coor(2)])
    % hold on
    % plot(centers(k,1),centers(k,2),'*')
end
title("Bezier Curve Approximated with Arc Spline")
xlabel('x coordinate')
ylabel('y coordinate')
axis equal





%Calculate the difference between points
% xy = [diff(x') diff(y')];
% lengths = sqrt(sum(xy.^2,2));
% figure;
% plot(t(1:end-1),lengths)
% title('Lengths along the curve')
% xlabel('t')
% ylabel('Lengths')