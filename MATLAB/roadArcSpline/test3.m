% Define the command to run the Python script
% & 'C:\ProgramData\anaconda3\envs\ox\python.exe' 'c:\Users\Atakan\.vscode\extensions\ms-python.python-2023.22.1\pythonFiles\lib\python\debugpy\adapter/../..\debugpy\launcher' '59943' '--' 'C:\Users\Atakan\Documents\GitHub\thesis\PYTHON\osm_to_csv.py';

lat1 = 33.78640;
lat2 = 33.78200;
lon1 = -84.38479;
lon2 = -84.38237;
fileName = "Test";
pythonCommand = ...
    sprintf('activate ox && python C:\\Users\\Atakan\\Documents\\GitHub\\thesis\\PYTHON\\osm_to_csv.py %f %f %f %f %s'...
    , lat1, lat2, lon1, lon2, fileName);

status = system(pythonCommand);

