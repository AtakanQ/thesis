function [segments] = representRoad(xEast,...
    yNorth,theta,curvature,turning_centers,L,all_clothoids,lineCfg,arcCfg)
%REPRESENTROAD Summary of this function goes here
%   Every tangent and curvature at every point is known inside this
%   function. This function's purpose is to return segments as structs
%   indicating each segment's nature. Meaning, some segments may be
%   represented with multiple arcs while some segments may be represented
%   as a straight line.
%Atakan
numSegment = length(L);

%% Sample road segment definiton.
sampleStruct.type = 'undefined'; % Segment type (line,arc,arcs or empty)
sampleStruct.numArcs = 0; % Number of arcs. Zero if segment is line.
sampleStruct.arcCurvatures = []; % Curvatures of all arcs
sampleStruct.arcCenters = []; % turning centers as x,y pairs.
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
% figure;
% for j = 1:length(lineIndices)
%     curr_seg_num = lineIndices(j);
%     all_clothoids(curr_seg_num).plotPlain();
%     hold on
%     plot(segments(curr_seg_num).allX,segments(curr_seg_num).allY,'--','Color',[0 1 1])
% end
% title("Debugging Line Segments")
% axis equal
% DEBUG LINE SEGMENTS END
%% Fit arc segments
for i = 1:numSegment
    % This segment was a straight line. Pass this segment.
    if ismember(i,lineIndices) 
        continue
    end
    

    %Generate first arc
    curr_center = turning_centers(i,:);
    % mean_curvature = (curvature(i) + curvature(i+1)) /2;
    radius = abs(1/curvature(i));
    % radius = abs(1/mean_curvature);
    [curr_center] = findCenters([xEast(i) yNorth(i)],theta(i),curvature(i));
    startPoint = [xEast(i) yNorth(i)];
    endPoint = [xEast(i+1) yNorth(i+1)];
    % arcSegment_v4(turningCenter, radius, startPoint, endPoint)
    tempArcSegment = arcSegment_v4(curr_center,radius,startPoint,endPoint,sign(curvature(i)) );

    % Compute the error.
    % [rms_error, max_error] = computeSegmentError(measurement_xy,ground_truth_xy)
    measurement_xy = [tempArcSegment.allX tempArcSegment.allY];
    ground_truth_xy = [all_clothoids(i).allX' all_clothoids(i).allY'];

    [rms_error, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy);
    
    % 1 arc is enough. Take the segment and break the loop.
    if max_error < arcCfg.maximumDistance
        segments(i).type = 'arc'; % Segment type (line,arc,arcs or empty)
        segments(i).numArcs = 1; % Number of arcs. Zero if segment is line.
        segments(i).arcCurvatures = curvature(i); % Curvatures of all arcs
        segments(i).arcCenters = curr_center; % turning centers as x,y pairs.
        segments(i).intermediatePoints = []; % List of x,y pairs
        segments(i).segmentLength = 0; % Segment length from clothoid fitting
        segments(i).headingInitial = 0;
        segments(i).headingFinal = 0;
        segments(i).headingChange = 0; % Heading change from clothoid fitting
        segments(i).initialCurvature = curvature(i); % Initial curvature provided in the function
        segments(i).finalCurvature = curvature(i); % Final curvature provided in the function
        segments(i).curvatureChange = 0; % Difference between initial and final curvatures
        segments(i).rmsError = rms_error;
        segments(i).maxError = max_error;
        segments(i).allX = tempArcSegment.allX;
        segments(i).allY = tempArcSegment.allY;
        continue
    end
    % figure;
    % tempArcSegment.plotArc();
    % all_clothoids(i).plotPlain();
    % If 1 arc was not enough, find where it exceeds the maximum
    % allowed error.
    if ~strcmp(segments(i).type,'arc')
        %At this stage there is 1 arc which does not meet requirements.
        segments(i).type = 'arcs';
        prevArcLen = 0;
        for j = 1:arcCfg.maximumNumArcs
            temp_idx = find( (errors > arcCfg.maximumDistance) );
            idx = temp_idx(1) - 1; % error is not breached at this index

            startPoint = [tempArcSegment.allX(idx) tempArcSegment.allY(idx)];
            endPoint = [xEast(i+1) yNorth(i+1)]; % try to generate until the end
            prevArcLen = prevArcLen + tempArcSegment.getArcLen(idx);
            len_ratio = prevArcLen / L(i); % this should be cumulative

            curr_curvature = curvature(i) + ...
                (curvature(i+1) - curvature(i)) * len_ratio;
            % mean_curvature = (curr_curvature + curvature(i+1))/2;
            % pts = [ tempArcSegment.allX(1) tempArcSegment.allY(1);...
            %     startPoint;
            %     endPoint];
            % [curvature_test, center_test] = findCurvature(pts);

            % curr_curvature = curvature_test; % TEST

            radius = abs(1/curr_curvature);

            lastTangent = tempArcSegment.tangents(idx);

            vector_angle_inward = lastTangent + sign(tempArcSegment.curv_sign)*pi/2;
            vector_inward = radius * [cos(vector_angle_inward) sin(vector_angle_inward)];
            curr_center = startPoint + vector_inward;
            % curr_center = center_test;
            %Generate first arc
            

            % arcSegment_v4(turningCenter, radius, startPoint, endPoint)
            tempArcSegment = arcSegment_v4(curr_center,...
                radius,startPoint,endPoint,sign(curvature(i)) );
            measurement_xy = [tempArcSegment.allX tempArcSegment.allY];

            [~, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy);
            % tempArcSegment.plotArc();
            if max_error < arcCfg.maximumDistance % error is small
                break
            end

        end
    end



end



end

