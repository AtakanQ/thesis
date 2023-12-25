function [lonlat_filename] = retrieveOSM(lat1, lat2, lon1, lon2, roadName,folderName)
lonlat_filename = [];
pythonCommand = ...
    sprintf('activate ox && python C:\\Users\\Atakan\\Documents\\GitHub\\thesis\\PYTHON\\osm_to_csv.py %f %f %f %f %s'...
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
for i = 3:length(files)
    % Check if the file name contains the specific string
    if (contains(files(i).name, roadName) && (files(i).name(1:3)~="nan"))
        % Check if the file size is larger than the current maximum
        if files(i).bytes > maxFileSize
            % Update the selected file information
            selectedFileName = files(i).name;
            maxFileSize = files(i).bytes;
        end
    end
end

% Display the result
if ~isempty(selectedFileName)
    fprintf('The selected road file is: %s\n', selectedFileName);
    fprintf('File size: %d bytes\n', maxFileSize);
else
    fprintf('No road file containing the specific road name was found.\n');
end
lonlat_filename = strcat(folderName,'\',selectedFileName);
end