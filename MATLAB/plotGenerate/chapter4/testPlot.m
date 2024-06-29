% Define the initial angle theta0
theta0 = 0; % Example initial angle in radians

% Define the length of the trajectory L
L = 100; % Example length

% Define the curvature function k(s)
% For a constant curvature rate:
k0 = 0.02; % Example constant curvature rate

% For a non-constant curvature rate, uncomment the following and comment out k0
k = @(s) k0 + 0.01 * s; % Example function of s

% Define the length of the trajectory
s = linspace(0, L, 1000); % Create an array of points from 0 to L

% Calculate the angle theta along the length for constant curvature
% theta_constant = theta0 + k0 * s;

% For non-constant curvature, use array integration
% Uncomment the following for non-constant curvature
theta_nonconstant = arrayfun(@(x) integral(k, 0, x), s);
theta_nonconstant = theta0 + theta_nonconstant;

% Plot the results
figure;
hold on;
% plot(s, theta_constant, 'b', 'LineWidth', 2);
% Uncomment the following for non-constant curvature
plot(s, theta_nonconstant, 'r--', 'LineWidth', 2);

xlabel('Length (L)');
ylabel('Angle (theta)');
title('Angle \theta over Length L');
% legend('Constant Curvature', 'Location', 'best');
% Uncomment the following for non-constant curvature
legend('Constant Curvature', 'Non-Constant Curvature', 'Location', 'best');
grid on;
hold off;

