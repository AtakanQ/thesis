function [res_clothoid] = concatenateClothoid(init_pos,init_tan, init_curvature, final_curvature,...
               cloth_length,groundX,groundY,errorCfg)
%CONCATENATECLOTHOID Summary of this function goes here
%   Detailed explanation goes here
arcSegClass = arcSegment;
maxOrder = 25;
ground_truth_xy = [groundX groundY];
res_clothoid = [];
for j = 1:maxOrder
    order = j;
    tempClothoid = clothoid(init_pos,init_tan, init_curvature, final_curvature,...
               cloth_length,order,arcSegClass);

    measurement_xy = [tempClothoid.allX' tempClothoid.allY'];

    [rms_error, max_error, errors] = ...
        computeSegmentError(measurement_xy,ground_truth_xy);
    if((rms_error < errorCfg.rmsError) && (max_error < errorCfg.maxError) )
        disp(['Clothoid segments ',num2str(start_idx),'-',num2str(end_idx), ' are concatenated with order: ', num2str(order)]  )
        disp(['RMS error: ',num2str(rms_error), ' Max error:',num2str(max_error)]  )
        res_clothoid = tempClothoid;
        break
    end


end


end

