function [segments] = representRoad(xEast,...
    yNorth,theta,curvature,L,lineCfg,all_clothoids)
%REPRESENTROAD Summary of this function goes here
%   Every tangent and curvature at every point is known inside this
%   function. This function's purpose is to return segments as structs
%   indicating each segment's nature. Meaning, some segments may be
%   represented with multiple arcs while some segments may be represented
%   as a straight line.
%Atakan
numSegment = length(L);

sampleStruct.type = 'undefined'; % Segment type (line,arc,arcs or empty)
sampleStruct.numArcs = 0; % Number of arcs. Zero if segment is line.
sampleStruct.arcCurvatures = []; % Curvatures of all arcs
sampleStruct.intermediatePoints = []; % List of x,y pairs
sampleStruct.segmentLength = 0; % Segment length from clothoid fitting
sampleStruct.headingInitial = 0;
sampleStruct.headingFinal = 0;
sampleStruct.headingChange = 0; % Heading change from clothoid fitting
sampleStruct.initialCurvature = 0; % Initial curvature provided in the function
sampleStruct.finalCurvature = 0; % Final curvature provided in the function
sampleStruct.curvatureChange = 0; % Difference between initial and final curvatures
sampleStruct.rmsError = 0;
sampleStruct.maxError = 0;
sampleStruct.allX = [];
sampleStruct.allY = [];

% Create an array of structures with empty fields
segments(numSegment+1) = sampleStruct;
segments(numSegment+1) = [];
%% Fit line segments first. Check the error and heading change.
lineIndices = [];
%DEBUG
% max_errors = zeros(numSegment,1);
% rms_errors = zeros(numSegment,1);
%DEBUG
for i = 1:numSegment
    heading_change = rad2deg(theta(i+1) - atan2(yNorth(i + 1 )-yNorth(i),...
        (xEast(i+1)-xEast(i)) ) );

    if (heading_change < lineCfg.lineDegreeDeviation) % Line does not make the segment deviate
        % Generate 500 points between the two coordinates.
        x_values = linspace(xEast(i), xEast(i+1) , lineCfg.numberOfPoints)';
        y_values = linspace(yNorth(i), yNorth(i+1), lineCfg.numberOfPoints)';
        heading = atan2(y_values(end)-y_values(1),x_values(end)-x_values(1));
        [rms_error, max_error] = computeSegmentError([x_values y_values],...
            [all_clothoids(i).allX' all_clothoids(i).allY']);
        %DEBUG
        % rms_errors(i) = rms_error;
        % max_errors(i) = max_error;
        %DEBUG
        if( (rms_error < lineCfg.rmsThreshold) && (max_error < lineCfg.maximumAllowedDistance))
            %This segment can be represented by a line.
            lineIndices = [lineIndices i]; % Remember this to use in the future.

            segments(i).type = 'line';
            segments(i).numArcs = 0;
            segments(i).arcCurvatures = []; % Curvatures of all arcs
            segments(i).intermediatePoints = []; % List of x,y pairs
            segments(i).segmentLength = norm([x_values(end)-x_values(1) y_values(end)-y_values(1)]); % Segment length from clothoid fitting
            segments(i).headingInitial = heading;
            segments(i).headingFinal = heading;
            segments(i).headingChange = 0; % Heading change from clothoid fitting
            segments(i).initialCurvature = 0; % Initial curvature provided in the function
            segments(i).finalCurvature = 0; % Final curvature provided in the function
            segments(i).curvatureChange = 0; % Difference between initial and final curvatures
            segments(i).allX = x_values;
            segments(i).allY = y_values;
            sampleStruct.rmsError = rms_error;
            sampleStruct.maxError = max_error;
        end
    end
end
% DEBUG LINE SEGMENTS START
figure;
for j = 1:length(lineIndices)
    curr_seg_num = lineIndices(j);
    all_clothoids(curr_seg_num).plotPlain();
    hold on
    plot(segments(curr_seg_num).allX,segments(curr_seg_num).allY,'--','Color',[0 1 1])
end
title("Debugging Line Segments")
axis equal
% DEBUG LINE SEGMENTS END

end

