function result = cumulativeSum(input_vector)
    % Initialize the output vector with the first element of the input vector
    result = zeros(length(input_vector),1);
    result(1) = input_vector(1);
    
    % Iterate through the input vector and perform the cumulative sum
    for i = 2:length(input_vector)
        % Add the current element to the previous cumulative sum
        result(i) = result(i-1) + input_vector(i);
    end
end