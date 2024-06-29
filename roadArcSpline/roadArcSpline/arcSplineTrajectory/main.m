close all
clear
%% Define points

pts = [5 5; 25 30; 55 55; 110 100;120 115; 125 130; 135 160];
x = pts(:,1); 
y = pts(:,2); 
[myRadii, centers] = findRadii(pts);
figure;
plot(pts(:,1), pts(:,2))
hold on
plot(pts(:,1), pts(:,2),'*')
title('Plot of Hand Given Data Points')
hold all; 
theta = linspace(0, 2*pi, 1000);
for i = 1:length(centers)
    centerX = centers(i,1);
    centerY = centers(i,2);
    radius = 1/myRadii(i);
    if radius < 1/0.001
        curr_point = [x(i+1) y(i+1)];

        crc_x = centerX + radius * cos(theta);
        crc_y = centerY + radius * sin(theta);
        plot(crc_x, crc_y, 'b', 'LineWidth', 0.2);
        plot(curr_point(1), curr_point(2), '*','LineWidth',1,'Color',[0 1 0]);
        plot([x(i) x(i+2)], [y(i) y(i+2)], '*','LineWidth',1,'Color',[0 0 1]);
        axis equal;
        hold on
    end
end

%% Define triangle
if 0
trng_pts = [50 10; 30 20; 40 30];

myRadii_trng = findRadii(trng_pts);
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


myRadii = findRadii([x ;y]');
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
end
%% Define Beziér Curve

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

% xy_diff = [diff(x') diff(y')];
% xy_rotation = zeros(length(xy_diff),2);
% for j = 1:length(curvature)
%     if curvature(j) > 0 %rotate cw
%         xy_rotation(j,:) = [xy_diff(j,2) -xy_diff(j,1)];
%     else %rotate ccw
%         xy_rotation(j,:) = [-xy_diff(j,2) xy_diff(j,1)];
%     end
% end



figure;
t = 0:0.001:1;
plot(t(1:5:end),curvature)
title("Generated Bezier Curve's Curvature (Analytical)")
xlabel('t')
ylabel('Curvature')


[bezierCurvature, centers] = findRadii([x' y']);

figure;
plot(x,y, 'LineWidth', 1,'Color',[1 0 0])
title("Generated Sample Bezier Curve")
hold on
theta = linspace(0, 2*pi, 1000);
for i = 1:length(centers)
    centerX = centers(i,1);
    centerY = centers(i,2);
    radius = 1/bezierCurvature(i);
    if(radius < 1/0.04)
        curr_point = [x(i+1) y(i+1)];

        crc_x = centerX + radius * cos(theta);
        crc_y = centerY + radius * sin(theta);
        plot(crc_x, crc_y, 'b', 'LineWidth', 0.2);
        plot(curr_point(1), curr_point(2), '*','LineWidth',1,'Color',[0 1 0]);
        plot([x(i) x(i+2)], [y(i) y(i+2)], '*','LineWidth',1,'Color',[0 0 1]);
        axis equal;
        hold on
    end
end


figure;
plot(linspace(0,1,length(bezierCurvature)),bezierCurvature)
title("Generated Bezier Curve's Curvature (MVRC method)")
xlabel('t')
ylabel('Curvature')





%Calculate the difference between points
% xy = [diff(x') diff(y')];
% lengths = sqrt(sum(xy.^2,2));
% figure;
% plot(t(1:end-1),lengths)
% title('Lengths along the curve')
% xlabel('t')
% ylabel('Lengths')