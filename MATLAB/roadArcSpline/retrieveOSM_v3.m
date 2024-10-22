function [xEast, yNorth,number_of_roads,refLat,refLon] = retrieveOSM_v3(lat1, lat2, lon1, lon2, roadName,folderName)
%This function is created to get rural road segments.
number_of_roads = 0;
all_lonlats = {};
all_lonlats_concatenated = [];

pythonCommand = ...
    sprintf('activate ox && python ..\\..\\PYTHON\\osm_to_csv_rural.py %f %f %f %f %s'...
    , lat1, lat2, lon1, lon2, folderName);
status = system(pythonCommand);

% Specify the directory path
directoryPath = strcat(folderName , '\\');

% Get a list of files in the directory
files = dir(directoryPath);

% Initialize variables to store information about the selected file
selectedFileName = '';
maxFileSize = 0;

% Loop through each file in the directory
csv_counter = 0;
% figure;
for i = 3:length(files)
    if (contains(files(i).name, roadName) && (files(i).name(1:3)~="nan"))
        csv_counter = csv_counter + 1;
        fileName = strcat( directoryPath,files(i).name );
        all_lonlats{csv_counter} = readCSV(fileName);
        all_lonlats_concatenated = [all_lonlats_concatenated; all_lonlats{csv_counter}];

        if(i == 3)
            refLat = mean(all_lonlats{csv_counter}(:,2));
            refLon = mean(all_lonlats{csv_counter}(:,1));
        end

        [xEast{csv_counter}, yNorth{csv_counter}, ~] = geodetic2enu(all_lonlats{csv_counter}(:,2), ...
            all_lonlats{csv_counter}(:,1), 0, refLat, refLon, 0, wgs84Ellipsoid);

        % plot(xEast{csv_counter},yNorth{csv_counter})
        % hold on
        % axis equal
    end

end

%% Parameters and variables
distanceThreshold = 1;
start_idx = 1;

%% start from first index's first point
foundIt = false;
thisSegmentIsConnected = false;

updateIdx = start_idx;
tempX = xEast{start_idx}(1);
tempY = yNorth{start_idx}(1);
distanceThreshold = 1;
index_array_from_first_point = [start_idx];
is_reversed_first_point = [0];
foundOneSegment = false;
while(~foundIt)
    thisSegmentIsConnected = false;
    for i = 1:csv_counter
        if i == updateIdx
            continue
        end
        currX = xEast{i}(1);
        currY = yNorth{i}(1);
        tempDist = norm( [currX currY] - [tempX tempY] );
        if (tempDist < distanceThreshold) % found it
            index_array_from_first_point = [index_array_from_first_point i];
            is_reversed_first_point = [is_reversed_first_point 0];
            tempX = xEast{i}(end);
            tempY = yNorth{i}(end);
            updateIdx = i;
            thisSegmentIsConnected = true;
            break
        else % try the other end
            currX = xEast{i}(end);
            currY = yNorth{i}(end);
            tempDist = norm( [currX currY] - [tempX tempY] );

            if( tempDist < distanceThreshold)
                index_array_from_first_point = [index_array_from_first_point i];
                is_reversed_first_point = [is_reversed_first_point 1];
                tempX = xEast{i}(1);
                tempY = yNorth{i}(1);
                updateIdx = i;
                thisSegmentIsConnected = true;
                break
            else
                thisSegmentIsConnected = false;
            end
        end

    end
    
    if(~thisSegmentIsConnected)
        foundIt = true;
    end

end

%% Start from first segments end
foundIt = false;
thisSegmentIsConnected = false;

updateIdx = start_idx;
tempX = xEast{start_idx}(end);
tempY = yNorth{start_idx}(end);

index_array_from_end_point = [];
is_reversed_end_point = [];
foundOneSegment = false;
while(~foundIt)
    thisSegmentIsConnected = false;
    for i = 1:csv_counter
        if i == updateIdx
            continue
        end
        currX = xEast{i}(1);
        currY = yNorth{i}(1);
        tempDist = norm( [currX currY] - [tempX tempY] );
        if (tempDist < distanceThreshold) % found it
            index_array_from_end_point = [index_array_from_end_point i];
            is_reversed_end_point = [is_reversed_end_point 0];
            tempX = xEast{i}(end);
            tempY = yNorth{i}(end);
            updateIdx = i;
            thisSegmentIsConnected = true;
            break
        else % try the other end
            currX = xEast{i}(end);
            currY = yNorth{i}(end);
            tempDist = norm( [currX currY] - [tempX tempY] );

            if( tempDist < distanceThreshold)
                index_array_from_end_point = [index_array_from_end_point i];
                is_reversed_end_point = [is_reversed_end_point 1];
                tempX = xEast{i}(1);
                tempY = yNorth{i}(1);
                updateIdx = i;
                thisSegmentIsConnected = true;
                break
            else
                thisSegmentIsConnected = false;
            end
        end

    end
    
    if(~thisSegmentIsConnected)
        foundIt = true;
    end

end


%% Merge everything
finalX = [];
finalY = [];
% Merge roads starting from first point
for n = 1:length(index_array_from_first_point)
    idx = index_array_from_first_point(n);
    if(is_reversed_first_point(n) == 0) %directly add
        finalX = [finalX; xEast{idx}];
        finalY = [finalY; yNorth{idx}];
    else % its flipped
        finalX = [finalX; flip(xEast{idx})];
        finalY = [finalY; flip(yNorth{idx})];
    end
end

% Merge roads starting from end point
for n = 1:length(index_array_from_end_point)
    idx = index_array_from_end_point(n);
    if(is_reversed_end_point(n) == 1) %directly add
        finalX = [xEast{idx}; finalX];
        finalY = [yNorth{idx}; finalY];
    else % its flipped
        finalX = [flip(xEast{idx}); finalX];
        finalY = [flip(yNorth{idx}); finalY];
    end
end

% Display the result
if (length(index_array_from_first_point) ~= 1) && (~isempty(index_array_from_end_point))
    fprintf('Found %d csv files\n', (csv_counter - 1));
    fprintf('Extracted road with %d segments\n', (length(index_array_from_first_point)+length(index_array_from_end_point)));
end
% [~, ia, ~] = unique(finalX,'first');

diffArr = diff(finalX);
firstDuplicateIndices = find(diffArr == 0) + 1;
finalX(firstDuplicateIndices) = [];
finalY(firstDuplicateIndices) = [];
xEast = finalX;
yNorth = finalY;

% xEast = finalX(ia);
% yNorth = finalY(ia);

number_of_roads = length(index_array_from_first_point) + length(index_array_from_end_point);

end