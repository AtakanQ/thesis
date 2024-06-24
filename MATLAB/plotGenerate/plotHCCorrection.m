close all
%% case 1
% Define the given parameters
L = 10;      % Length (you can adjust this value as needed)
k_ri = 3;    % Starting value for the first line
k_rf = 7;    % Ending value for both lines
k_i = 1;     % Starting value for the second line

% Define the points for the lines
x = [0, L];
y1 = [k_ri, k_rf];
y2 = [k_i, k_rf];

% Create the figure
figure;
hold on;

xp = [0, L, 0];
yp = [k_i, k_rf, k_ri];
fill(xp, yp, [0 0.6 0],'FaceAlpha',0.5); % 'g' specifies the color green
centroidX = mean(xp);
centroidY = mean(yp);

% Add text inside the polygon
text(centroidX, centroidY, 'A', 'HorizontalAlignment', 'center', 'FontSize', 13);
% Plot the first line
plot(x, y1, 'b', 'LineWidth', 2.5);

% Plot the second line
plot(x, y2, 'r--', 'LineWidth', 2.5);


% Set axis limits
xlim([0 L*1.1]);
ylim([-k_i k_rf*1.1]);

% Remove numbers from the axes
set(gca, 'XTick', [0 L]);
set(gca, 'YTick', [0 k_i k_ri k_rf]);

yticklabels({'0','k_i','k_r_i','k_r_f'})
xticklabels({'0','L'})

% Label the important points
% text(0, k_ri, 'k_{ri}', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12);
% text(0, k_i, 'k_{i}', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12);
% text(L, k_rf, 'k_{rf}', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12);
% text(0, 0, '0', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 12);
% text(L, 0, 'L', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 12);

% Add axis labels using the given letters
xlabel('Arc Length (m)',"FontSize",13);
ylabel('Curvature (m^-^1)',"FontSize",13);
title("Road Curvature and Desired Vehicle Curvature","FontSize",13)


% Display the grid for reference
grid on;

% Plot dummy points for the legend
h1 = plot(nan, nan, 'b', 'LineWidth', 1.2);
h2 = plot(nan, nan, 'r', 'LineWidth', 1.2);

% Create a legend with only the dummy points
legend([h1, h2], {'Road Curvature', 'Vehicle Curvature'}, 'Location', 'Best');

% Hold off to stop adding more plots
hold off;
%% case 2
% Define the given parameters
L = 10;      % Length (you can adjust this value as needed)
L1 = 6;      % Length (you can adjust this value as needed)
k_ri = 3;    % Starting value for the first line
k_rf = 7;    % Ending value for both lines
k_i = 5;     % Starting value for the second line
k_intermediate = 4;
% Define the points for the lines
x = [0, L];
y = [k_ri, k_rf];

x1 = [0, L1];
y1 = [k_i, k_intermediate];

x2 = [L1, L];
y2 = [k_intermediate, k_rf];

% Create the figure
figure;
hold on;


xp = [0, 60/17, 0];
yp = [k_i, 75/17, k_ri];
fill(xp, yp, [0 0.6 0],'FaceAlpha',0.5); % 'g' specifies the color green
centroidX = mean(xp);
centroidY = mean(yp);
% Add text inside the polygon
text(centroidX, centroidY, 'A_1', 'HorizontalAlignment', 'center', 'FontSize', 13);

xp = [60/17, 6, 6];
yp = [75/17, 27/5, 4];
fill(xp, yp, [0 0.6 0],'FaceAlpha',0.5); % 'g' specifies the color green
centroidX = mean(xp);
centroidY = mean(yp);
% Add text inside the polygon
text(centroidX, centroidY, 'A_2', 'HorizontalAlignment', 'center', 'FontSize', 13);

xp = [6, 6, 10];
yp = [4, 27/5, 7];
fill(xp, yp, [0 0.6 0],'FaceAlpha',0.5); % 'g' specifies the color green
centroidX = mean(xp);
centroidY = mean(yp);
% Add text inside the polygon
text(centroidX, centroidY, 'A_3', 'HorizontalAlignment', 'center', 'FontSize', 13);


% Plot the first line road
plot(x, y, 'b', 'LineWidth', 2.5);

% Plot the second line first man
plot(x1, y1, 'r--', 'LineWidth', 2.5);
% Plot the second line first man
plot(x2, y2, 'r--', 'LineWidth', 2.5);

% Set axis limits
xlim([0 L*1.1]);
ylim([-k_i k_rf*1.1]);

% Remove numbers from the axes
set(gca, 'XTick', [0 L1, L]);
set(gca, 'YTick', [0 k_ri k_intermediate k_i  k_rf]);

yticklabels({'0','k_r_i','k_i_n_t','k_i','k_r_f'})
xticklabels({'0','l1','L'})

% Label the important points
% text(0, k_ri, 'k_{ri}', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12);
% text(0, k_i, 'k_{i}', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12);
% text(L, k_rf, 'k_{rf}', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12);
% text(0, 0, '0', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 12);
% text(L, 0, 'L', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 12);

% Add axis labels using the given letters
xlabel('Arc Length (m)',"FontSize",13);
ylabel('Curvature (m^-^1)',"FontSize",13);
title("Road Curvature and Desired Vehicle Curvature","FontSize",13)


% Display the grid for reference
grid on;

% Plot dummy points for the legend
h1 = plot(nan, nan, 'b', 'LineWidth', 1.2);
h2 = plot(nan, nan, 'r', 'LineWidth', 1.2);

% Create a legend with only the dummy points
legend([h1, h2], {'Road Curvature', 'Vehicle Curvature'}, 'Location', 'Best');

% Hold off to stop adding more plots
hold off;
