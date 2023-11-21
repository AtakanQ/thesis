function [errors] = computeError(arcSegments,all_clothoids)

%Number of arc segments is one lesser than clothoids. TODO
numSegments = length(arcSegments);
errors = cell(numSegments,1);
num_coor_arc = length(arcSegments{1}.Coordinates);

% figure;
for j = 1:numSegments
    curr_clothoid_points = [all_clothoids(j).allX' all_clothoids(j).allY'];
    errors{j} = zeros(num_coor_arc,1);
    for k = 1:num_coor_arc
        curr_point = arcSegments{j}.Coordinates(k,:);
        
        % Calculate Euclidean distances between 'curr_point' and all points in 'curr_clothoid_points'
        distances = sqrt(sum(bsxfun(@minus, curr_clothoid_points, curr_point).^2, 2));
        
        % Find the index of the closest point
        [minError, ~] = min(distances);
        errors{j}(k) = minError;
        % 
        % plot(curr_point(1),curr_point(2),'*')
        % hold on
        % plot(curr_clothoid_points(:,1),curr_clothoid_points(:,2),'.','MarkerSize',15)
    end
end

end

