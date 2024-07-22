function [otherLanes,xEast,yNorth] = generateOtherLanes(segments,laneWidth,numLanes)
% Assume left most lane is given.
for i = 1:length(segments)
    if(segments(i).type == "clothoid")
        break
    end
end
otherLanes = cell(1,numLanes);

%% Sample struct
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
sampleStruct.computationTime = 0;
% Create an array of structures with empty fields
numSegment = length(segments);
for i = 1:numLanes
    otherLanes{i}(numSegment+1) = sampleStruct;
    otherLanes{i}(numSegment+1) = [];
end
xEast = cell(1,numLanes);
yNorth = cell(1,numLanes);

arcSegClass = arcSegment;

%% Start generating
for i = 1:numLanes
    xEast{i} = [];
    yNorth{i} = [];
    % figure;
    for j = 1:length(segments)
        if(segments(j).type == "clothoid")
            %positive curvature turns left.
            %Assume left lane is given.
            
            angles = findArcAngles(segments(j));
            new_radii = abs(1./segments(j).arcCurvatures) + ...
                sign(segments(j).arcCurvatures)*i*laneWidth;
            new_length = sum(angles .* new_radii);
            
            original_init_pos = [segments(j).allX(1) segments(j).allY(1)];

            init_pos_shift = sign(segments(j).initialCurvature)*i*laneWidth*...
                (original_init_pos - segments(j).arcCenters(1,:))./...
                norm(original_init_pos - segments(j).arcCenters(1,:));

            init_pos = original_init_pos + init_pos_shift;

            init_tan = segments(j).headingInitial;
            new_init_curvature = sign(segments(j).initialCurvature)/...
                (abs(1/segments(j).initialCurvature) + ...
                sign(segments(j).initialCurvature)*i*laneWidth);
            new_final_curvature = sign(segments(j).finalCurvature)/...
                (abs(1/segments(j).finalCurvature) + ...
                sign(segments(j).finalCurvature)*i*laneWidth);
            
            order = segments(j).numArcs;
            tempClothoid = clothoid(init_pos,init_tan, new_init_curvature, new_final_curvature,...
               new_length,order,arcSegClass);

            xEast{i} = [xEast{i}; tempClothoid.allX'];
            yNorth{i} = [yNorth{i}; tempClothoid.allY'];

            otherLanes{i}(j).type = 'clothoid';
            otherLanes{i}(j).numArcs = order;
            otherLanes{i}(j).arcCurvatures = tempClothoid.curvatures; % Curvatures of all arcs
            otherLanes{i}(j).segmentLength = tempClothoid.length; % Segment length from clothoid fitting
            otherLanes{i}(j).headingInitial = tempClothoid.init_tan;
            otherLanes{i}(j).headingFinal = tempClothoid.final_tangent;
            otherLanes{i}(j).headingChange = tempClothoid.final_tangent - tempClothoid.init_tan; % Heading change from clothoid fitting
            otherLanes{i}(j).initialCurvature = tempClothoid.init_curv; % Initial curvature provided in the function
            otherLanes{i}(j).finalCurvature = tempClothoid.final_curv; % Final curvature provided in the function
            otherLanes{i}(j).curvatureChange = tempClothoid.final_curv - tempClothoid.init_curv; % Difference between initial and final curvatures
            otherLanes{i}(j).curvatureDerivative = otherLanes{i}(j).curvatureChange/tempClothoid.length;
            otherLanes{i}(j).allX = tempClothoid.allX;
            otherLanes{i}(j).allY = tempClothoid.allY;
            otherLanes{i}(j).computationTime = tempClothoid.computationTime;
            
            % if i == 2
            %    idx = 1;
            % elseif i == 1
            %    idx = 2;
            % end

            % if( (j+1) <= numel(all_clothoids{idx})) % if this index exists.
            %     measurement_xy = [tempClothoid.allX' tempClothoid.allY'];
            %     ground_truth_xy = [all_clothoids{idx}(j+1).allX' all_clothoids{idx}(j+1).allY'];
            % 
            %     [rms_error, max_error, ~] = ...
            %         computeSegmentError(measurement_xy,ground_truth_xy);
            % 
            %     otherLanes{i}(j).rmsError = rms_error;
            %     otherLanes{i}(j).maxError = max_error;
            % else %if it does not exist, error cannot be computed.
                otherLanes{i}(j).rmsError = NaN;
                otherLanes{i}(j).maxError = NaN;
            % end
            % tempClothoid.plotPlain();

            % hold on

        elseif(segments(j).type == "line")

            original_init_pos = [segments(j).allX(1) segments(j).allY(1)];

            init_pos_shift = i*laneWidth*...
                [sin(segments(j).headingInitial) -cos(segments(j).headingInitial) ];

            init_pos = original_init_pos + init_pos_shift;
            
            tic
            line_allX = segments(j).allX + init_pos_shift(1);
            line_allY = segments(j).allY + init_pos_shift(2);
            compTime = toc;

            tan = segments(j).headingInitial;
            
            otherLanes{i}(j).type = 'line';
            otherLanes{i}(j).numArcs = 0;
            otherLanes{i}(j).arcCurvatures = []; % Curvatures of all arcs
            otherLanes{i}(j).segmentLength = segments(j).segmentLength; % Segment length from clothoid fitting
            otherLanes{i}(j).headingInitial = segments(j).headingInitial;
            otherLanes{i}(j).headingFinal = segments(j).headingFinal;
            otherLanes{i}(j).headingChange = 0; % Heading change from clothoid fitting
            otherLanes{i}(j).initialCurvature = 0; % Initial curvature provided in the function
            otherLanes{i}(j).finalCurvature = 0; % Final curvature provided in the function
            otherLanes{i}(j).curvatureChange = 0; % Difference between initial and final curvatures
            otherLanes{i}(j).curvatureDerivative = 0;
            otherLanes{i}(j).allX = line_allX;
            otherLanes{i}(j).allY = line_allY;
            otherLanes{i}(j).computationTime = compTime;

            %exception happens to be here as well
            % if i == 2
            %    idx = 1;
            % elseif i == 1
            %    idx = 2;
            % end

            % if( (j+1) <= numel(all_clothoids{idx})) % if this index exists.
            % measurement_xy = [line_allX line_allY];
            % ground_truth_xy = [all_clothoids{idx}(j).allX' all_clothoids{idx}(j).allY'];
            % 
            % [rms_error, max_error, ~] = ...
            %     computeSegmentError(measurement_xy,ground_truth_xy);
            % 
            %     otherLanes{i}(j).rmsError = rms_error;
            %     otherLanes{i}(j).maxError = max_error;
            % else %if it does not exist, error cannot be computed.
                otherLanes{i}(j).rmsError = NaN;
                otherLanes{i}(j).maxError = NaN;
            % end

            xEast{i} = [xEast{i}; line_allX];
            yNorth{i} = [yNorth{i}; line_allY];
            
        end

    end
end

end