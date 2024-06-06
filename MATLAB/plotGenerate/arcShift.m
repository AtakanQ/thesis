% Define parameters
center = [0, 0]; % Center of the arcs
radius1 = 10; % Radius of the first arc (original lane)
radius2 = radius1 + 3.7; % Radius of the second arc (shifted lane)
theta = linspace(0, pi/2, 100); % Define the angle from 0 to 90 degrees

% Calculate coordinates for the first arc
x_arc1 = center(1) + radius1 * cos(theta);
y_arc1 = center(2) + radius1 * sin(theta);

% Calculate coordinates for the second arc
x_arc2 = center(1) + radius2 * cos(theta);
y_arc2 = center(2) + radius2 * sin(theta);

% Calculate a point in the middle of the arcs for the quiver
mid_theta = pi/4; % Midpoint angle for demonstration (45 degrees)
x_quiver_start = center(1) + radius1 * cos(mid_theta);
y_quiver_start = center(2) + radius1 * sin(mid_theta);
x_quiver_end = center(1) + radius2 * cos(mid_theta);
y_quiver_end = center(2) + radius2 * sin(mid_theta);

% Open a new figure
figure;

% Plot the first arc
plot(x_arc1, y_arc1, 'b', 'LineWidth', 2);
hold on;

% Plot the second arc
plot(x_arc2, y_arc2, 'r', 'LineWidth', 2);

% Plot the radial difference with quiver
quiver(x_quiver_start, y_quiver_start, x_quiver_end - x_quiver_start, y_quiver_end - y_quiver_start, 0, 'k', 'LineWidth', 1, 'MaxHeadSize', 0.5);

% Annotate the radial difference
text((x_quiver_start + x_quiver_end)/2, (y_quiver_start + y_quiver_end)/2, sprintf('3.7 units'), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');

% Set up plot limits
axis equal;
xlim([min([x_arc1, x_arc2])-1, max([x_arc1, x_arc2])+1]);
ylim([min([y_arc1, y_arc2])-1, max([y_arc1, y_arc2])+1]);

% Add labels and title
xlabel('xEast (m)','FontSize',13);
ylabel('yNorth (m)','FontSize',13);
title('Parallel Shifting of an Arc','FontSize',13);

% Add grid
grid on;

% Add legend
legend('Original Lane', 'Shifted Lane', 'Radial Difference');
