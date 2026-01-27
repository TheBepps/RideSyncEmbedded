%% CSV TO MAT CONVERTER (Optimized)
clear; clc;

% --- FILE NAMES ---
inputFile  = 'Global_ft2a.csv';  
outputFile = 'Global_ft2a.mat';  

% Check if file exists
if ~isfile(inputFile)
    error('File %s not found in the current folder.', inputFile);
end

%% 1. DATA LOADING
fprintf('Reading CSV file (this might take a while)...\n');

opts = detectImportOptions(inputFile);
opts.VariableNamingRule = 'preserve'; 
T = readtable(inputFile, opts);

fprintf('Data loaded: %d rows, %d columns.\n', height(T), width(T));

%% 2. STRUCTURE OPTIMIZATION (Timetable)
fprintf('Converting to Timetable for advanced analysis...\n');

% Find the time column
timeColName = '';
possibleNames = {'Time', 'time', 't', 'Tempo'};
for i = 1:length(possibleNames)
    if ismember(possibleNames{i}, T.Properties.VariableNames)
        timeColName = possibleNames{i};
        break;
    end
end

if ~isempty(timeColName)
    % Remove duplicate time rows to ensure strict monotonicity
    [~, uniqueIdx] = unique(T.(timeColName));
    T = T(uniqueIdx, :);
    
    % --- FIX: CONVERT DOUBLE TO DURATION ---
    % table2timetable requires 'duration' or 'datetime', not 'double'.
    % We assume the time column is in seconds.
    T.(timeColName) = seconds(T.(timeColName));
    
    % Create Timetable
    TT = table2timetable(T, 'RowTimes', timeColName);
    
    GlobalData = TT; 
    fprintf('Structure successfully converted to TIMETABLE.\n');
else
    warning('Time column not found. Saving as standard table.');
    GlobalData = T;
end

%% 3. SAVE .MAT
fprintf('Saving binary .mat file...\n');

save(outputFile, 'GlobalData', '-v7.3'); % -v7.3 ensures support for files >2GB

fprintf('------------------------------------------------\n');
fprintf('DONE! File saved as: %s\n', outputFile);
fprintf('------------------------------------------------\n');
fprintf('To load data:\n');
fprintf('>> load(''%s'')\n', outputFile);
fprintf('To visualize:\n');
fprintf('>> stackedplot(GlobalData)\n');