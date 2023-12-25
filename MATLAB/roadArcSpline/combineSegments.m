function [result_clothoids,concat_indices_clothoid,result_lines,concat_indices_line] = combineSegments(segments,all_clothoids,errorCfg)
%errorTol is the maximum allowed difference between curvature derivatives
%in percentage
result_clothoids = [];
result_lines = [];
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
for j = 1:length(concat_indices_clothoid)
    start_idx = concat_indices_clothoid{j}(1);
    end_idx   = concat_indices_clothoid{j}(end);


    init_pos = [segments(start_idx).allX(1) segments(start_idx).allY(1)];
    init_tan = segments(start_idx).headingInitial;
    init_curvature = segments(start_idx).initialCurvature;
    final_curvature = segments(end_idx).finalCurvature;
    cloth_length = 0;
    
    for k = 0:(length(concat_indices_clothoid) - 1)
        cloth_length = cloth_length + segments(start_idx+k).segmentLength;
    end

    order = ceil(  (segments(start_idx).numArcs+ segments(end).numArcs ) /2 );
    order = 10;
    % clothoid(init_pos,init_tan, init_curvature, final_curvature,...
    %            length,order,arcSegClass)
    tempClothoid = clothoid(init_pos,init_tan, init_curvature, final_curvature,...
               cloth_length,order,arcSegClass);

    measurement_xy = [tempClothoid.allX' tempClothoid.allY'];
    groundX = [];
    groundY = [];
    figure;
    tempClothoid.plotPlain();
    axis equal
    for n = 0:(length(concat_indices_clothoid) - 1)
        groundX = [groundX; all_clothoids(start_idx+n).allX' ];
        groundY = [groundY; all_clothoids(start_idx+n).allY' ];
        hold on
        all_clothoids(start_idx+n).plotPlain();
        
    end
    title(strcat('Real and concatenated curves ',num2str(start_idx), ' and ',num2str(end_idx)))
    xlabel('m')
    ylabel('m')

    [res_clothoid] = concatenateClothoid(init_pos,init_tan, init_curvature, final_curvature,...
               cloth_length,groundX,groundY,errorCfg);

    result_clothoids = [result_clothoids res_clothoid];

    figure;
    plot(errors)
    title(strcat('Error along curve for concatenation indices:', num2str(start_idx),' and ',...
        num2str(end_idx )) )
    xlabel('index')
    ylabel('Error (m)')

    
end

for j = 1:length(concat_indices_line)
    start_idx = concat_indices_line{j}(1);
    end_idx   = concat_indices_line{j}(end);
    numSegments = length(concat_indices_line{j});

    xVal = linspace( segments(start_idx).allX(1), segments(end_idx).allX(end))';
    yVal = linspace( segments(start_idx).allY(1), segments(end_idx).allY(end))';

    measurement_xy = [xVal yVal];

    groundX = [];
    groundY = [];
    figure;
    plot(xVal,yVal,'--','LineWidth',1.5,'Color',[0 0 1]);
    axis equal
    for n = 0:(numSegments - 1)
        groundX = [groundX; all_clothoids(start_idx+n).allX' ];
        groundY = [groundY; all_clothoids(start_idx+n).allY' ];
        hold on
        all_clothoids(start_idx+n).plotPlain();
    end
    title(strcat('Real and concatenated curves (for line segment) ',num2str(start_idx), ' and ',num2str(end_idx)))
    xlabel('m')
    ylabel('m')
    legend('Concatenated Line','Ground Truth Clothoid')

    ground_truth_xy = [groundX groundY];
    [rms_error, max_error, errors] = ...
        computeSegmentError(measurement_xy,ground_truth_xy);
    if((rms_error < errorCfg.rmsError) && (max_error < errorCfg.maxError) )
        disp(['Line segments ',num2str(start_idx),'-',num2str(end_idx), ' are concatenated']  )
        disp(['RMS error: ',num2str(rms_error), ' Max error:',num2str(max_error)]  )

        lineStruct.allX = xVal;
        lineStruct.allY = yVal;
        result_lines = [result_lines lineStruct];
    end
    figure;
    plot(errors)
    title(strcat('Error along curve for concatenation indices:', num2str(start_idx),' and ',...
        num2str(end_idx )) )
    xlabel('index')
    ylabel('Error (m)')
end


end

