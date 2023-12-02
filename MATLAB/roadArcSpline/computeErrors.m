function [all_errors, rms_errors] = computeErrors(arcSegments,all_clothoids)

numSegments = length(arcSegments);
all_errors = cell(numSegments,1);
num_coor_arc = length(arcSegments{1}.Coordinates);
rms_errors = zeros(numSegments,1);
% DEBUG
% figure;
for j = 1:numSegments
    curr_clothoid_points = [all_clothoids(j).allX' all_clothoids(j).allY'];
    all_errors{j} = zeros(num_coor_arc,1);
    for k = 1:num_coor_arc
        curr_point = arcSegments{j}.Coordinates(k,:);
        
        % Calculate Euclidean distances between 'curr_point' and all points in 'curr_clothoid_points'
        distances = sqrt(sum(bsxfun(@minus, curr_clothoid_points, curr_point).^2, 2));
        
        % Find the minimum of the closest point
        [minError, ~] = min(distances);
        all_errors{j}(k) = minError;
        % DEBUG
        % plot(curr_point(1),curr_point(2),'*')
        % hold on
        % plot(curr_clothoid_points(:,1),curr_clothoid_points(:,2),'.','MarkerSize',15)
    end

    rms_errors(j) = rms(all_errors{j});
end

end

