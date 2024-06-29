P1 = [0, 0];  % initial point
P2 = [5, 5];  % final point

% Define the tangents at the initial and final points
tangent1 = [0; 0.01];  % tangent at initial point
tangent2 = [0; 1];  % tangent at final point

Rccw = [cos(theta), -sin(theta); sin(theta), cos(theta)];

slope1 = sign(curv1) * Rccw * tangent1;