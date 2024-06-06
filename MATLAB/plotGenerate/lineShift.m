% Define parameters
length = 7; % Length of the line
angle_degrees = 60; % Angle in degrees
shift_distance = 3.7; % Perpendicular shift distance

% Calculate starting point and ending point of the first line
x_start = 0;
y_start = 0;
x_end = length * cosd(angle_degrees);
y_end = length * sind(angle_degrees);

% Coordinates for the first line
x_line1 = [x_start, x_end];
y_line1 = [y_start, y_end];

% Calculate the direction vector for the first line
direction_vector = [x_end - x_start, y_end - y_start];
% Calculate a perpendicular vector to the direction vector
perpendicular_vector = [-direction_vector(2), direction_vector(1)];
% Normalize the perpendicular vector
perpendicular_vector = perpendicular_vector / norm(perpendicular_vector);

% Calculate the start and end points of the shifted line
x_shifted_start = x_start + shift_distance * perpendicular_vector(1);
y_shifted_start = y_start + shift_distance * perpendicular_vector(2);
x_shifted_end = x_end + shift_distance * perpendicular_vector(1);
y_shifted_end = y_end + shift_distance * perpendicular_vector(2);

% Coordinates for the shifted line
x_line2 = [x_shifted_start, x_shifted_end];
y_line2 = [y_shifted_start, y_shifted_end];

% Open a new figure
figure;

% Plot the first line
plot(x_line1, y_line1, 'b', 'LineWidth', 2);
hold on;

% Plot the shifted line
plot(x_line2, y_line2, 'r', 'LineWidth', 2);

% Plot the shifting direction and distance
quiver( (x_start+x_end)/2, (y_start + y_end)/2, shift_distance * perpendicular_vector(1), shift_distance * perpendicular_vector(2), 0, 'k', 'LineWidth', 1, 'MaxHeadSize', 0.5);

% Annotate the shifting distance
mid_point_shift = [(x_start + x_shifted_start)/2, (y_start + y_shifted_start)/2];
text(mid_point_shift(1), mid_point_shift(2), sprintf('Shift: %.1f units', shift_distance), 'HorizontalAlignment', 'center');

% Set up plot limits
axis equal;
xlim([min([x_line1, x_line2])-1, max([x_line1, x_line2])+1]);
ylim([min([y_line1, y_line2])-1, max([y_line1, y_line2])+1]);

% Add labels and title
xlabel('xEast (m)','FontSize',13);
ylabel('yNorth (m)','FontSize',13);
title('Parallel Shift For Lines','FontSize',13);

% Add grid
grid on;

% Add legend
legend('Original Lane', 'Shifted Lane', 'Direction of Shift');
