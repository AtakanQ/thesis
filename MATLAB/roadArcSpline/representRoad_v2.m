function [segments] = representRoad_v2(xEast,...
    yNorth,theta,curvature,L,all_clothoids,lineCfg,arcCfg,DEBUG)
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
sampleStruct.segmentLength = 0; % Segment length from clothoid fitting
sampleStruct.headingInitial = 0;
sampleStruct.headingFinal = 0;
sampleStruct.headingChange = 0; % Heading change from clothoid fitting
sampleStruct.initialCurvature = 0; % Initial curvature provided in the function
sampleStruct.finalCurvature = 0; % Final curvature provided in the function
sampleStruct.curvatureChange = 0; % Difference between initial and final curvatures
sampleStruct.curvatureDerivative = 0;
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
            segments(i).segmentLength = norm([x_values(end)-x_values(1) y_values(end)-y_values(1)]); % Segment length from clothoid fitting
            segments(i).headingInitial = heading;
            segments(i).headingFinal = heading;
            segments(i).headingChange = 0; % Heading change from clothoid fitting
            segments(i).initialCurvature = 0; % Initial curvature provided in the function
            segments(i).finalCurvature = 0; % Final curvature provided in the function
            segments(i).curvatureChange = 0; % Difference between initial and final curvatures
            segments(i).curvatureDerivative = 0;
            segments(i).allX = x_values;
            segments(i).allY = y_values;
            segments(i).rmsError = rms_error;
            segments(i).maxError = max_error;
            
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
arcSegClass = arcSegment(); %dummy arcSeg
for i = 1:numSegment
    % This segment was a straight line. Pass this segment.
    if ismember(i,lineIndices) 
        continue
    end
    curr_curvature = curvature(i);
    next_curvature = curvature(i+1);
    curr_len = L(i);
    startPoint = [xEast(i) yNorth(i)];
    init_tan = theta(i);
    ground_truth_xy = [all_clothoids(i).allX' all_clothoids(i).allY'];
    segments(i).type = 'clothoid';

    % clothoid(init_pos,init_tan, init_curvature, final_curvature,...
    %                length,order,arcSegClass)
    tempClothoid = clothoid(startPoint,init_tan,curr_curvature,next_curvature...
        ,curr_len,1,arcSegClass);
    
    measurement_xy = [tempClothoid.allX' tempClothoid.allY'];
    
    [rms_error, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy);
    
    % if i == 28
    %     DEBUG = true;
    % else
    %     DEBUG = false;
    % end

    if DEBUG
        figure;
        plot(errors)
        title( strcat(['Euclidian Distance Error For Initial Try' ...
            ' Segment Number:'],num2str(i)  ) )
        xlabel('Data index')
        ylabel('Error (m)')

        figure;
        tempClothoid.plotPlain();
        % tempClothoid.plotClothoidWithCircles();
        hold on
        all_clothoids(i).plotPlain();
        title( strcat('Trajectory For Initial Try Segment Number:',num2str(i)  ) )
        legend("tempClothoid","Ground Truth")
        axis equal
    end

    % error is low. try fewer arcs started with 5
    if(max_error < arcCfg.maximumDistance)
        for j = (arcCfg.initialTry-1):-1:1
            nextTempClothoid = clothoid(startPoint,init_tan,curr_curvature,next_curvature...
                ,curr_len,j,arcSegClass);
            nextMeasurement_xy = [nextTempClothoid.allX' nextTempClothoid.allY'];
            [nextRms_error, nextMax_error, nextErrors] = ...
                computeSegmentError(nextMeasurement_xy,ground_truth_xy);

            tempClothoid = nextTempClothoid;

            segments(i).numArcs = j;
            segments(i).allX = tempClothoid.allX;
            segments(i).allY = tempClothoid.allY;
            segments(i).arcCenters = tempClothoid.centers;
            segments(i).arcCurvatures = tempClothoid.curvatures;
            segments(i).segmentLength = tempClothoid.length;
            segments(i).headingInitial = tempClothoid.init_tan;
            segments(i).headingFinal = tempClothoid.final_tangent;
            segments(i).headingChange = tempClothoid.final_tangent - tempClothoid.init_tan;
            segments(i).initialCurvature = tempClothoid.init_curv;
            segments(i).finalCurvature = tempClothoid.final_curv;
            segments(i).curvatureChange = tempClothoid.final_curv - tempClothoid.init_curv;
            segments(i).curvatureDerivative = segments(i).curvatureChange/tempClothoid.length;
            segments(i).rmsError = nextRms_error;
            segments(i).maxError = nextMax_error;

            if(nextMax_error < arcCfg.maximumDistance)
                % keep decreasing
            else
                % take the previous clothoid and leave
                break
            end
        end
    else % increase the number of arcs
        for j = (arcCfg.initialTry+1):1:arcCfg.maximumNumArcs
            nextTempClothoid = clothoid(startPoint,init_tan,curr_curvature,next_curvature...
                ,curr_len,j,arcSegClass);
            nextMeasurement_xy = [nextTempClothoid.allX' nextTempClothoid.allY'];
            [nextRms_error, nextMax_error, nextErrors] = ...
            computeSegmentError(nextMeasurement_xy,ground_truth_xy);

            tempClothoid = nextTempClothoid;

            if(nextMax_error > arcCfg.maximumDistance)
                %error is still bad
                %try increasing more
                
            else
                % take the clothoid and leave
                segments(i).numArcs = j;
                segments(i).allX = tempClothoid.allX;
                segments(i).allY = tempClothoid.allY;
                segments(i).arcCenters = tempClothoid.centers;
                segments(i).arcCurvatures = tempClothoid.curvatures;
                segments(i).segmentLength = tempClothoid.length;
                segments(i).headingInitial = tempClothoid.init_tan;
                segments(i).headingFinal = tempClothoid.final_tangent;
                segments(i).headingChange = tempClothoid.final_tangent - tempClothoid.init_tan;
                segments(i).initialCurvature = tempClothoid.init_curv;
                segments(i).finalCurvature = tempClothoid.final_curv;
                segments(i).curvatureChange = tempClothoid.final_curv - tempClothoid.init_curv;
                segments(i).curvatureDerivative = segments(i).curvatureChange/tempClothoid.length;
                segments(i).rmsError = nextRms_error;
                segments(i).maxError = nextMax_error;
                break
            end
        end
    end

    if DEBUG
        figure;
        plot(nextErrors)
        title( strcat('Euclidian Distance Error For Segment Number:',num2str(i), ' Order Became',num2str(j)  ) )
        xlabel('Data index')
        ylabel('Error (m)')

        figure;
        tempClothoid.plotPlain();
        % tempClothoid.plotClothoidWithCircles();
        hold on
        all_clothoids(i).plotPlain();
        title( strcat('Trajectory For Segment Number:',num2str(i), ' Order Became',num2str(j)  ) )
        legend("tempClothoid","Ground Truth")
        axis equal
    end


    end



end


