function [res_clothoid,rms_error,max_error] = concatenateClothoid(init_pos,init_tan, init_curvature, final_curvature,...
               cloth_length,groundX,groundY,errorCfg)
%CONCATENATECLOTHOID Summary of this function goes here
%   Detailed explanation goes here
arcSegClass = arcSegment;
maxOrder = 10;
ground_truth_xy = [groundX groundY];
res_clothoid = [];
rms_error = Inf;
max_error = Inf;
for j = 1:maxOrder
    order = j;
    tempClothoid = clothoid(init_pos,init_tan, init_curvature, final_curvature,...
               cloth_length,order,arcSegClass);

    measurement_xy = [tempClothoid.allX' tempClothoid.allY'];

    [rms_error, max_error, errors] = ...
        computeSegmentError(measurement_xy,ground_truth_xy);
    if((rms_error < errorCfg.rmsError) && (max_error < errorCfg.maxError) )
        res_clothoid = tempClothoid;
        break
    end


end


end

