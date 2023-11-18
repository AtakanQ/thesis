function [estimated_length] = findSegmentLen(init_tan,init_curv,final_tan,final_curv)
% Assume that the curve is clothoid. Since the curvature changes linearly
% we can just use the average curvature to find the segment length using
% the below formula. Also it is known that clothoids can be approximated
% with 5 arcs with equal length. Assume that curvature does not change
% sign.
% SegmentLength * Curvature = AngleChange
deltaAngle = abs(final_tan-init_tan);
average_curvature = (init_curv + final_curv)/2;

estimated_length = abs(deltaAngle / average_curvature);
end

