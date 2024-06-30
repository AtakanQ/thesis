function [result_clothoids,concat_indices_clothoid,result_lines,concat_indices_line,mergedSegments] = ...
    combineSegments(segments,all_clothoids,errorCfg)
%errorTol is the maximum allowed difference between curvature derivatives
%in percentage
result_clothoids = [];
result_lines = [];
result_residual_lines = [];
maximumNumConcat = 5; %  Maximum 5 clothoids can be concatenated
i = 1;
concat_indices_clothoid = {};
concat_counter_clothoid = 1;
concat_indices_line = {};
concat_counter_line = 1;
while (i < numel(segments)) % keep the current segment.
    if ( ~isempty(segments(i).numArcs)) % exists
        isClothoid = (segments(i).type == "clothoid"); % it is clothoid
    else %fitting was failed, continue
        i = i+1;
        continue;
    end
    

    concat_list_clothoid = i;
    concat_list_line = i;
    if(isClothoid) % clothoid
        curr_curv_derivative = segments(i).curvatureDerivative;
        for j = 1:maximumNumConcat
            if( (i + j) < numel(segments) && ( ~isempty(segments(i + j).numArcs)))
                next_curv_derivative =  segments(i + j).curvatureDerivative;
    
                %do not take absolute value here!
                %derivative of the curvature must not change sign!
                ratio = next_curv_derivative / curr_curv_derivative;
    
                %in the range
                if ( (ratio > (1-errorCfg.errorTol)) && (ratio < (1+errorCfg.errorTol)) )
                    concat_list_clothoid = [concat_list_clothoid i+j]; % append the current 
                else
                    break
                end
            else
                break
            end
        end
    
        if(numel(concat_list_clothoid) > 1) % smth is concatenated
            concat_indices_clothoid{concat_counter_clothoid} = concat_list_clothoid;
            concat_counter_clothoid = concat_counter_clothoid + 1;
            i = i+j+1;
        else % nothing is concatenated
            i = i+1;
        end
    else % line
        init_tan = segments(i).headingInitial; % initial headin
        for j = 1:maximumNumConcat
            % if the next element exists, 
            % if it is successfuly created, 
            % if it is a line
            if( (i + j) < numel(segments) && ( ~isempty(segments(i + j).numArcs)) && (segments(i+j).type == "line") )
                next_init_tan =  segments(i + j).headingInitial;
    
                %do not take absolute value here!
                %derivative of the curvature must not change sign!
                heading_diff = rad2deg(abs(next_init_tan - init_tan));
    
                %in the range
                if ( heading_diff < errorCfg.headingDeviation )
                    concat_list_line = [concat_list_line i+j]; % append the current 
                else
                    break
                end
            else
                break
            end
        end
    
        if(numel(concat_list_line) > 1) % smth is concatenated
            concat_indices_line{concat_counter_line} = concat_list_line;
            concat_counter_line= concat_counter_line + 1;
            i = i+j+1;
        else % nothing is concatenated
            i = i+1;
        end
    end

end

disp(strcat(num2str(concat_counter_clothoid - 1)," candidate combination (for clothoids) will be tried.",...
    "Here are the segment indices:") )
disp(concat_indices_clothoid)

disp(strcat(num2str(concat_counter_line - 1)," candidate combination (for lines) will be tried.",...
    "Here are the segment indices:") )
disp(concat_indices_line)
%% Try concatenating and check error.
arcSegClass = arcSegment;
concat_successful_indices_clothoid = {};
cloth_counter = 0;
rms_errors_clothoid = [];
max_errors_clothoid = [];
for j = 1:length(concat_indices_clothoid)
    start_idx = concat_indices_clothoid{j}(1);
    end_idx   = concat_indices_clothoid{j}(end);


    init_pos = [segments(start_idx).allX(1) segments(start_idx).allY(1)];
    init_tan = segments(start_idx).headingInitial;
    init_curvature = segments(start_idx).initialCurvature;
    final_curvature = segments(end_idx).finalCurvature;
    cloth_length = 0;
    
    for k = 0:(length(concat_indices_clothoid{j}) - 1)
        cloth_length = cloth_length + segments(start_idx+k).segmentLength;
    end

    groundX = [];
    groundY = [];
    for n = 0:(length(concat_indices_clothoid) - 1)
        groundX = [groundX; all_clothoids(start_idx+n).allX' ];
        groundY = [groundY; all_clothoids(start_idx+n).allY' ];
    end

    [res_clothoid,rms_error,max_error] = concatenateClothoid(init_pos,init_tan, init_curvature, final_curvature,...
               cloth_length,groundX,groundY,errorCfg);
    if( ~isempty(res_clothoid)) %successfuly concatenated
        result_clothoids = [result_clothoids res_clothoid];
        cloth_counter = cloth_counter + 1;
        concat_successful_indices_clothoid{cloth_counter} = start_idx:end_idx;
        rms_errors_clothoid(cloth_counter) = rms_error;
        max_errors_clothoid(cloth_counter) = max_error;
        disp(['Clothoid segments ',num2str(start_idx),'-',num2str(end_idx), ' are concatenated with order: ', num2str(res_clothoid.order)]  )
        disp(['RMS error: ',num2str(rms_error), ' Max error:',num2str(max_error)]  )
    end
    
end

concat_successful_indices_line = {};
line_counter = 0;
rms_errors_line = [];
max_errors_line = [];

concat_successful_residual_indices_line = {};
residual_line_counter = 0;
rms_errors_residual_line = [];
max_errors_residual_line = [];

for j = 1:length(concat_indices_line)
    start_idx = concat_indices_line{j}(1);
    for k = 0:(length(concat_indices_line{j}) - 2)
        
        end_idx   = concat_indices_line{j}(end - k);
        numSegments = end_idx - start_idx + 1;
    
        % xVal = linspace( segments(start_idx).allX(1), segments(end_idx).allX(end),numSegments*500)';
        % yVal = linspace( segments(start_idx).allY(1), segments(end_idx).allY(end),numSegments*500)';
        
        segLength = norm([segments(start_idx).allX(1)-segments(end_idx).allX(end) ...
            segments(start_idx).allY(1)-segments(end_idx).allY(end)]);
        xVal = linspace( segments(start_idx).allX(1), segments(end_idx).allX(end),segLength/0.01)';
        yVal = linspace( segments(start_idx).allY(1), segments(end_idx).allY(end),segLength/0.01)';

        measurement_xy = [xVal yVal];
    
        groundX = [];
        groundY = [];
        for n = 0:(numSegments - 1)
            groundX = [groundX; all_clothoids(start_idx+n).allX' ];
            groundY = [groundY; all_clothoids(start_idx+n).allY' ];
            % hold on
            % all_clothoids(start_idx+n).plotPlain();
        end
    
        ground_truth_xy = [groundX groundY];
        [rms_error, max_error, ~] = ...
            computeSegmentError(measurement_xy,ground_truth_xy);
        if((rms_error < errorCfg.rmsError) && (max_error < errorCfg.maxError) )
            disp(['Line segments ',num2str(start_idx),'-',num2str(end_idx), ' are concatenated']  )
            disp(['RMS error: ',num2str(rms_error), ' Max error:',num2str(max_error)]  )
            line_counter = line_counter + 1;
            concat_successful_indices_line{line_counter} = start_idx:end_idx;
            rms_errors_line(line_counter) = rms_error;
            max_errors_line(line_counter) = max_error;
            lineStruct.allX = xVal;
            lineStruct.allY = yVal;
            result_lines = [result_lines lineStruct];
            break
        end
    end

    % if concatenation did not reach end
    if k ~= 0
        % try to concatenate the rest
        start_idx = concat_indices_line{j}(end - k + 1);
        for rest_counter = 0:(k - 2)
            end_idx   = concat_indices_line{j}(end - rest_counter);
            numSegments = end_idx - start_idx + 1;
        
            xVal = linspace( segments(start_idx).allX(1), segments(end_idx).allX(end),numSegments*500)';
            yVal = linspace( segments(start_idx).allY(1), segments(end_idx).allY(end),numSegments*500)';
        
            measurement_xy = [xVal yVal];
        
            groundX = [];
            groundY = [];
            for n = 0:(numSegments - 1)
                groundX = [groundX; all_clothoids(start_idx+n).allX' ];
                groundY = [groundY; all_clothoids(start_idx+n).allY' ];
                % hold on
                % all_clothoids(start_idx+n).plotPlain();
            end
        
            ground_truth_xy = [groundX groundY];
            [rms_error, max_error, ~] = ...
                computeSegmentError(measurement_xy,ground_truth_xy);
            if((rms_error < errorCfg.rmsError) && (max_error < errorCfg.maxError) )
                disp(['Line segments ',num2str(start_idx),'-',num2str(end_idx), ' are concatenated [residual]']  )
                disp(['RMS error: ',num2str(rms_error), ' Max error:',num2str(max_error)]  )
                residual_line_counter = residual_line_counter + 1;
                concat_successful_residual_indices_line{residual_line_counter} = start_idx:end_idx;
                rms_errors_residual_line(residual_line_counter) = rms_error;
                max_errors_residual_line(residual_line_counter) = max_error;
                lineStruct.allX = xVal;
                lineStruct.allY = yVal;
                result_residual_lines = [result_residual_lines lineStruct];
                break
            end
        end
    end
end



%% Sample road segment definiton.
% Merge the segments and return the merged version
mergedSegmentCounter = 0;
i = 1;
while(i <= length(segments))
    mergedSegmentCounter = mergedSegmentCounter + 1;

    isConcatenatedLine = false;
    isConcatenatedClothoid = false;
    isConcatenatedResidualLine = false;
    for j = 1:length(concat_successful_indices_line) 
        if (concat_successful_indices_line{j}(1) == i)
            isConcatenatedLine = true;
            break
        end
    end
    if(~isConcatenatedLine)
        for k = 1:length(concat_successful_indices_clothoid)
            if (concat_successful_indices_clothoid{k}(1) == i)
                isConcatenatedClothoid = true;
                break
            end
        end
    end
    
    if(~isConcatenatedLine && ~isConcatenatedClothoid)
        for n = 1:length(concat_successful_residual_indices_line)
            if (concat_successful_residual_indices_line{n}(1) == i)
                isConcatenatedResidualLine = true;
                break
            end
        end
    end

    if(isConcatenatedLine) % it is a concatenated line
        sampleStruct.type = 'line'; % Segment type (line,arc,arcs or empty)
        sampleStruct.numArcs = 0; % Number of arcs. Zero if segment is line.
        sampleStruct.arcCurvatures = []; % Curvatures of all arcs
        sampleStruct.arcCenters = []; % turning centers as x,y pairs.
        sampleStruct.segmentLength = norm([result_lines(j).allX(1)-result_lines(j).allX(end)...
            result_lines(j).allY(1)-result_lines(j).allY(end)]); % Segment length from clothoid fitting
        sampleStruct.headingInitial = atan2( result_lines(j).allY(end) - result_lines(j).allY(1), ...
            result_lines(j).allX(end) - result_lines(j).allX(1));
        sampleStruct.headingFinal = sampleStruct.headingInitial;
        sampleStruct.headingChange = 0; % Heading change from clothoid fitting
        sampleStruct.initialCurvature = 0; % Initial curvature provided in the function
        sampleStruct.finalCurvature = 0; % Final curvature provided in the function
        sampleStruct.curvatureChange = 0; % Difference between initial and final curvatures
        sampleStruct.curvatureDerivative = 0;
        sampleStruct.rmsError = rms_errors_line(j);
        sampleStruct.maxError = max_errors_line(j);
        sampleStruct.allX = result_lines(j).allX;
        sampleStruct.allY = result_lines(j).allY;
        mergedSegments(mergedSegmentCounter) = sampleStruct;

        i = i + length(concat_successful_indices_line{j});

    elseif(isConcatenatedResidualLine) % it is a concatenated residual line
        sampleStruct.type = 'line'; % Segment type (line,arc,arcs or empty)
        sampleStruct.numArcs = 0; % Number of arcs. Zero if segment is line.
        sampleStruct.arcCurvatures = []; % Curvatures of all arcs
        sampleStruct.arcCenters = []; % turning centers as x,y pairs.
        sampleStruct.segmentLength = norm([result_residual_lines(n).allX(1)-result_residual_lines(n).allX(end)...
            result_residual_lines(n).allY(1)-result_residual_lines(n).allY(end)]); % Segment length from clothoid fitting
        sampleStruct.headingInitial = atan2( result_residual_lines(n).allY(end) - result_residual_lines(n).allY(1), ...
            result_residual_lines(n).allX(end) - result_residual_lines(n).allX(1));
        sampleStruct.headingFinal = sampleStruct.headingInitial;
        sampleStruct.headingChange = 0; % Heading change from clothoid fitting
        sampleStruct.initialCurvature = 0; % Initial curvature provided in the function
        sampleStruct.finalCurvature = 0; % Final curvature provided in the function
        sampleStruct.curvatureChange = 0; % Difference between initial and final curvatures
        sampleStruct.curvatureDerivative = 0;
        sampleStruct.rmsError = rms_errors_residual_line(n);
        sampleStruct.maxError = max_errors_residual_line(n);
        sampleStruct.allX = result_residual_lines(n).allX;
        sampleStruct.allY = result_residual_lines(n).allY;
        mergedSegments(mergedSegmentCounter) = sampleStruct;

        i = i + length(concat_successful_residual_indices_line{n});
    elseif(isConcatenatedClothoid) % it is a concatenated clothoid
        tempClothoid = result_clothoids(k);
        sampleStruct.type = 'clothoid'; % Segment type (line,arc,arcs or empty)
        sampleStruct.numArcs = tempClothoid.order; % Number of arcs. Zero if segment is line.
        sampleStruct.arcCurvatures = tempClothoid.curvatures; % Curvatures of all arcs
        sampleStruct.arcCenters = tempClothoid.centers; % turning centers as x,y pairs.
        sampleStruct.segmentLength = tempClothoid.length; % Segment length from clothoid fitting
        sampleStruct.headingInitial = tempClothoid.init_tan;
        sampleStruct.headingFinal = tempClothoid.final_tangent;
        sampleStruct.headingChange = tempClothoid.final_tangent - tempClothoid.init_tan; % Heading change from clothoid fitting
        sampleStruct.initialCurvature = tempClothoid.init_curv; % Initial curvature provided in the function
        sampleStruct.finalCurvature = tempClothoid.final_curv; % Final curvature provided in the function
        sampleStruct.curvatureChange = tempClothoid.final_curv - tempClothoid.init_curv; % Difference between initial and final curvatures
        sampleStruct.curvatureDerivative = sampleStruct.curvatureChange/sampleStruct.segmentLength;
        sampleStruct.rmsError = rms_errors_clothoid(k);
        sampleStruct.maxError = max_errors_clothoid(k);
        sampleStruct.allX = tempClothoid.allX;
        sampleStruct.allY = tempClothoid.allY;
        mergedSegments(mergedSegmentCounter) = sampleStruct;
        
        i = i + length(concat_successful_indices_clothoid{k});
    else % it is not concatenated
        mergedSegments(mergedSegmentCounter) = segments(i);
        i = i + 1;
    end


end

end

