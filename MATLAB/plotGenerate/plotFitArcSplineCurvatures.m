%% Plot k0 and theta0
%theta0 is negative and k0 is positive

% Parameters for the clothoid
a = 0.001;    % This constant determines the rate at which curvature changes
L = 50;     % Total arc length

% Generate arc length values from -L/2 to L/2
s = linspace(0, L, 1000);

k1 = -0.01;
% Calculate the curvature k(s) = a * s
k = k1 + a * s;

% Create the plot
figure;
plot(s, k/2, 'LineWidth', 2);
grid on;
xlabel('Arc Length, s','FontSize',13);
ylabel('Curvature, k','FontSize',13);
title('Curvature Error Correction with a Clothoid','FontSize',13);

% Annotating the plot with k1, k2, and L
hold on;

% Points to annotate k_1 and k_2
k1 = k(1);
k2 = k(end);

% Plotting points
plot(0, k1/2, 'ro'); % Point at start
plot(L, k2/2, 'ro');  % Point at end

% Text annotations
% text(0, k1, '  k_1', 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'right', 'FontSize', 12);
% text(0, k2, '  k_2', 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'right', 'FontSize', 12);
text(L, 0, '  L', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 13);

% Adjusting plot limits if necessary
ylim([-1.1*max(k), 1.1*max(k)]);
xlim([0 L*1.5])
% Remove all ticks and labels
set(gca, 'XTick', [0 L], 'YTick', [k1/2 k2/2]);
yticklabels({'k_0','0'})
xticklabels({'0',''})
set(gca, 'FontSize', 13);
hold off;   


