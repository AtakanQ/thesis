function [result] = combineSegments(segments,all_clothoids,errorTol)
%errorTol is the maximum allowed difference between curvature derivatives
%in percentage
result = [];
maximumNumConcat = 5; %  Maximum 5 clothoids can be concatenated
i = 1;
concat_indices = {};
concat_counter = 1;
while (i < numel(segments)) % keep the current segment.
    curr_curv_derivative = segments(i).curvatureDerivative;

    concat_list = i;

    for j = 1:maximumNumConcat
        if( (i + j) < numel(segments))
            next_curv_derivative =  segments(i + j).curvatureDerivative;

            %do not take absolute value here!
            %derivative of the curvature must not change sign!
            ratio = next_curv_derivative / curr_curv_derivative;

            %in the range
            if ( (ratio > (1-errorTol)) && (ratio < (1+errorTol)) )
                concat_list = [concat_list i+j]; % append the current 
            else
                break
            end
        else
            break
        end
    end

    if(numel(concat_list) > 1) % smth is concatenated
        concat_indices{concat_counter} = concat_list;
        concat_counter = concat_counter + 1;
        i = i+j+1;
    else % nothing is concatenated
        i = i+1;
    end

end

disp(strcat(num2str(concat_counter - 1)," candidate combination will be tried.",...
    "Here are the segment indices:") )
disp(concat_indices)
%% Try concatenating and check error.
arcSegClass = arcSegment;
for j = 1:length(concat_indices)
    start_idx = concat_indices{j}(1);
    end_idx   = concat_indices{j}(end);


    init_pos = [segments(start_idx).allX(1) segments(start_idx).allY(1)];
    init_tan = segments(start_idx).headingInitial;
    init_curvature = segments(start_idx).initialCurvature;
    final_curvature = segments(end_idx).finalCurvature;
    cloth_length = 0;
    
    for k = 0:(length(concat_indices) - 1)
        cloth_length = cloth_length + segments(start_idx+k).segmentLength;
    end

    order = ceil(  (segments(start_idx).numArcs+ segments(end).numArcs ) /2 );
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
    for n = 0:(length(concat_indices) - 1)
        groundX = [groundX; all_clothoids(start_idx+n).allX' ];
        groundY = [groundY; all_clothoids(start_idx+n).allY' ];
        hold on
        all_clothoids(start_idx+n).plotPlain();
        
    end
    title('Real and concatenated curves')
    xlabel('m')
    ylabel('m')

    ground_truth_xy = [groundX groundY];
    [rms_error, max_error, errors] = ...
        computeSegmentError(measurement_xy,ground_truth_xy);
    figure;
    plot(errors)
    title(strcat('Error along curve for concatenation indices:', num2str(start_idx),' and ',...
        num2str(end_idx )) )
    xlabel('index')
    ylabel('Error (m)')

    
end

end

