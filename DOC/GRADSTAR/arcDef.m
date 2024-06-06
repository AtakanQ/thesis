% Define the parameters
r = 5; % Radius of the arc
theta_start = pi/4; % Starting angle in radians, shifted by 90 degrees
theta_end = 3*pi/4; % Ending angle in radians, shifted by 90 degrees
center = [3, 2]; % Coordinates of the center of the circle

% Adjust angles for 90 degree rotation
theta_start = theta_start + 3*pi/2;
theta_end = theta_end + 3*pi/2;

% Calculate the coordinates of the arc
theta = linspace(theta_start, theta_end, 100); % Generate 100 points between the angles
x = center(1) + r*cos(theta);
y = center(2) + r*sin(theta);

% Calculate the coordinates for the full circle (for visualization)
theta_full = linspace(0, 2*pi, 100);
x_full = center(1) + r*cos(theta_full);
y_full = center(2) + r*sin(theta_full);

% Open a new figure
figure;

% Plot the full circle in light gray (for reference)
plot(x_full, y_full, 'Color', [0.8 0.8 0.8]);
hold on;

% Plot the arc
plot(x, y, 'b', 'LineWidth', 2);

% Mark the center of the circle
plot(center(1), center(2), 'ro', 'MarkerFaceColor', 'r');

% Draw radius lines from the center to the start and end of the arc
plot([center(1), center(1) + r*cos(theta_start)], [center(2), center(2) + r*sin(theta_start)], 'k--');
plot([center(1), center(1) + r*cos(theta_end)], [center(2), center(2) + r*sin(theta_end)], 'k--');

% Annotate the figure
text(center(1), center(2), 'Center', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
text(center(1) + r*cos((theta_start+theta_end)/2), center(2) + r*sin((theta_start+theta_end)/2), 'Arc', 'HorizontalAlignment', 'left');
text(center(1) + 0.5*r*cos(theta_start) - 0.1 , center(2) + 0.5*r*sin(theta_start), 'r', 'HorizontalAlignment', 'right');
text(center(1) + 0.15*r*cos((theta_start+theta_end)/2)*1.2, center(2) + 0.15*r*sin((theta_start+theta_end)/2)*1.2, '\theta', 'HorizontalAlignment', 'center');

% Draw the central angle using an arc
angle_arc = linspace(theta_start, theta_end, 20);
plot(center(1) + 0.15*r*cos(angle_arc), center(2) + 0.15*r*sin(angle_arc), 'r-');

axis equal;
title('Representation of an Arc');
