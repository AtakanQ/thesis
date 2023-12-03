function [rms_error, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy)

num_coor_arc = length(measurement_xy);
errors = zeros(num_coor_arc,1);
for k = 1:num_coor_arc
    curr_point = measurement_xy(k,:);
    
    % Calculate Euclidean distances between 'curr_point' and all points in 'curr_clothoid_points'
    distances = sqrt(sum(bsxfun(@minus, ground_truth_xy, curr_point).^2, 2));
    
    % Find the minimum of the closest point
    [minError, ~] = min(distances);
    errors(k) = minError;
    % DEBUG
    % plot(curr_point(1),curr_point(2),'*')
    % hold on
    % plot(curr_clothoid_points(:,1),curr_clothoid_points(:,2),'.','MarkerSize',15)
end
rms_error = rms(errors);
max_error = max(errors);

end

