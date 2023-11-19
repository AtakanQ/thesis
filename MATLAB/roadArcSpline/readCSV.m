function lonlat = readCSV(filename, dataLines)
%IMPORTFILE Import data from a text file
%  O214 = IMPORTFILE(FILENAME) reads data from text file FILENAME for
%  the default selection.  Returns the numeric data.
%
%  O214 = IMPORTFILE(FILE, DATALINES) reads data for the specified row
%  interval(s) of text file FILENAME. Specify DATALINES as a positive
%  scalar integer or a N-by-2 array of positive scalar integers for
%  dis-contiguous row intervals.
%
%  Example:
%  O214 = importfile("C:\Users\Atakan\Desktop\thesis2\PYTHON\O-21___4.csv", [2, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 04-Nov-2023 08:58:02

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Latitude", "Longitude"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
O214 = readtable(filename, opts);

%% Convert to output type
lonlat = table2array(O214);
end

