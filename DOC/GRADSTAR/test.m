% Define a realistic highway curve
x = linspace(0, 10, 400);
y = 0.1 * x.^2;  % Parabolic curve representing a smooth highway

% Calculate the curvature at a specific point
dx = gradient(x);
dy = gradient(y);
ddx = gradient(dx);
ddy = gradient(dy);
curvature = abs(ddx .* dy - dx .* ddy) ./ (dx.^2 + dy.^2).^1.5;

% Select a point to show curvature
point_index = 150;
curv = curvature(point_index);

% Calculate the radius of curvature
radius_of_curvature = 1 / curv;

% Calculate the turning center (center of curvature)
tangent_vector = [dx(point_index), dy(point_index)];
normal_vector = [-tangent_vector(2), tangent_vector(1)];  % Perpendicular to the tangent
normal_vector = normal_vector / norm(normal_vector);  % Normalize

center_of_curvature = [x(point_index), y(point_index)] + radius_of_curvature * normal_vector;

% Plot the highway curve
figure;
plot(x, y, 'LineWidth', 2);
hold on;
scatter(x(point_index), y(point_index), 'filled', 'r');


% Plot the turning center
scatter(center_of_curvature(1), center_of_curvature(2), 'filled', 'b');
line([x(point_index), center_of_curvature(1)], [y(point_index), center_of_curvature(2)], 'Color', 'b', 'LineStyle', '--');

% Plot the quarter circle
theta = linspace(0, -pi/2, 100);
quarter_circle_x = center_of_curvature(1) + radius_of_curvature * cos(theta);
quarter_circle_y = center_of_curvature(2) + radius_of_curvature * sin(theta);
quiver(x(point_index),y(point_index),65*dx(point_index),65*dy(point_index),"LineWidth",2,"Color",[0.75 0.5 0.5],"MaxHeadSize",3.5)
plot(quarter_circle_x, quarter_circle_y, 'b--');

title('Waypoint Representation');
xlabel('X-axis');
ylabel('Y-axis');
legend('Highway Curve', 'Curvature Point', 'Turning Center', 'Curvature', 'Heading');
grid on;
axis equal;
hold off;
