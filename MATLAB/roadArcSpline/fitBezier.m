function [trajectories] = fitBezier(init_pos,init_tan,init_curv,clothoid_GT)
%FITBEZIER Summary of this function goes here
%   Detailed explanation goes here

A = init_pos;

B = [clothoid_GT.allX(end) clothoid_GT.allY(end)];


tzero = [cos(init_tan) sin(init_tan)]';

final_angle = clothoid_GT.allTangent(end);
tfinal = [cos(final_angle) sin(final_angle)]';

curvature_zero = init_curv;

curvature_final = clothoid_GT.allCurvature(end);

trajectories = bezier(A,B,tzero,tfinal,curvature_zero,curvature_final);

position_error_bezier = norm(trajectories.Curves{1}(end,:) - B)
heading_error_bezier = rad2deg(trajectories.Tangents{1}(end) - final_angle)
curvature_error_bezier = trajectories.Curvatures{1}(end) - curvature_final
end

