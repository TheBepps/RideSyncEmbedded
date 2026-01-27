%% 1. Setup and Initialization
clear; clc; close all;

% --- PATH DEFINITION ---
basePath = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\Final test\ft4\ft4b';
path_file1 = fullfile(basePath, 'ft4b1');
path_file2 = fullfile(basePath, 'ft4b2');

% --- FILE LISTING & SORTING ---
d1 = dir(fullfile(path_file1, '*.csv'));
d2 = dir(fullfile(path_file2, '*.csv'));

if isempty(d1) || isempty(d2), error('Files not found.'); end

[~, idx1] = sort({d1.name}); files1 = d1(idx1);
[~, idx2] = sort({d2.name}); files2 = d2(idx2);

N1 = length(files1);
N2 = length(files2);
nFiles = max(N1, N2);
fprintf('Detected segments: Ch1=%d, Ch2=%d. Starting Bulk Import (Standard Format)...\n', N1, N2);

%% 2. IMPORT CONFIGURATION (Native)
% Since data is now comma-separated with dot decimals, we use standard options.

% Config for Set 1
opts1 = detectImportOptions(fullfile(path_file1, files1(1).name));
opts1.Delimiter = ',';             % Updated separator
opts1.VariableNamesLine = 1;       % Header names
opts1.DataLines = [3 Inf];         % Skip Unit Row (Row 2) - Data starts at 3
opts1.PreserveVariableNames = true;

% Config for Set 2
opts2 = detectImportOptions(fullfile(path_file2, files2(1).name));
opts2.Delimiter = ',';
opts2.VariableNamesLine = 1;
opts2.DataLines = [3 Inf];
opts2.PreserveVariableNames = true;

%% 3. BULK LOADING (Stitching Files)

buffer1 = cell(nFiles, 1);
buffer2 = cell(nFiles, 1);

fprintf('Loading files into memory...\n');

% Use parfor for parallel processing
for k = 1:nFiles
    
    % --- PROCESS LIST 1 (PROTECTED) ---
    if k <= N1 
        f1 = fullfile(path_file1, files1(k).name);
        try
            % Read table using defined options
            T1 = readtable(f1, opts1);
            % Convert to matrix immediately
            buffer1{k} = T1{:,:}; 
        catch
            warning('Error reading file from List 1 at index %d', k);
        end
    end
    
    % --- PROCESS LIST 2 (PROTECTED) ---
    if k <= N2 
        f2 = fullfile(path_file2, files2(k).name);
        try
            % Read table using defined options
            T2 = readtable(f2, opts2);
            % Convert to matrix immediately
            buffer2{k} = T2{:,:}; 
        catch
            warning('Error reading file from List 2 at index %d', k);
        end
    end
end

% --- FINAL CLEANUP ---
% Remove empty cells from the buffers.
% (If N1 != N2, the buffer corresponding to the shorter list will have empty trailing cells)
buffer1 = buffer1(~cellfun('isempty', buffer1));
buffer2 = buffer2(~cellfun('isempty', buffer2));

fprintf('Import complete. Buffer1: %d segments, Buffer2: %d segments.\n', length(buffer1), length(buffer2));

fprintf('Concatenating segments...\n');

% Create the TWO GIANT MATRICES
BigM1 = vertcat(buffer1{:});
BigM2 = vertcat(buffer2{:});

% Free memory
clear buffer1 buffer2; 

fprintf('Data Loaded. Ref Points: %d, Sync Points: %d.\n', length(BigM1), length(BigM2));

%% 4. VARIABLE MAPPING (Global Vectors)

% File 1 (Reference) - Check column indices if your export changed order!
% Assuming: Time, ChA, ChB, ChC, ChD, Dig
t1_vec_raw  = BigM1(:, 1);
Vout        = BigM1(:, 2);
Enable      = BigM1(:, 3);
Vbatt_TEG   = BigM1(:, 4);
Vbatt_PV    = BigM1(:, 5);
D0_1        = BigM1(:, 6);

% File 2 (Target)
t2_vec_raw  = BigM2(:, 1);
ICurr_TEG   = BigM2(:, 2); 
V_TEG       = BigM2(:, 3);
ICurr_PV    = BigM2(:, 4); 
V_PV        = BigM2(:, 5);
D0_2        = BigM2(:, 6);

%% 5. TIME VECTOR CORRECTION (Sawtooth Fix)
% If files reset time to 0, this stitches them into one continuous timeline.
fprintf('Fixing time vectors...\n');
t1_vec = fixTimeVector(t1_vec_raw);
t2_vec = fixTimeVector(t2_vec_raw);

%% 6. GLOBAL SYNCHRONIZATION
fprintf('Analyzing Synchronization Signal (D0)...\n');
logic_thresh = 0.5;

% Find Global Start/End Indices
idx_s1 = find(D0_1 > logic_thresh, 1, 'first');
idx_e1 = find(D0_1 > logic_thresh, 1, 'last');
idx_s2 = find(D0_2 > logic_thresh, 1, 'first');
idx_e2 = find(D0_2 > logic_thresh, 1, 'last');

if isempty(idx_s1) || isempty(idx_e1) || isempty(idx_s2) || isempty(idx_e2)
    error('Could not find complete High-Low pulse in the merged data.');
end

% Get Timestamps
t1_start = t1_vec(idx_s1); t1_end = t1_vec(idx_e1);
t2_start = t2_vec(idx_s2); t2_end = t2_vec(idx_e2);

dur1 = t1_end - t1_start;
dur2 = t2_end - t2_start;

fprintf('Pulse Detected.\nRef Duration: %.4fs\nSync Duration: %.4fs\n', dur1, dur2);

% Calculate Sync Parameters
alpha = dur1 / dur2;              % Skew
beta  = t1_start - (alpha * t2_start);    % Offset

% Apply Correction to ENTIRE Target Time Vector
t2_corr = (t2_vec .* alpha) + beta;

%% 7. CUT & INTERPOLATE
fprintf('Interpolating and creating Final Matrix...\n');

% Define Master Indices (Active Region only)
master_indices = idx_s1:idx_e1;

% Slice Reference Data
t_final      = t1_vec(master_indices);
Vout_final   = Vout(master_indices);
Enable_final = Enable(master_indices);
VbattT_final = Vbatt_TEG(master_indices);
VbattP_final = Vbatt_PV(master_indices);

% Interpolate Target Data onto 't_final'
% Using 'linear' interp on the corrected time axis
ICurr_TEG_i = interp1(t2_corr, ICurr_TEG, t_final, 'linear', 'extrap');
V_TEG_i     = interp1(t2_corr, V_TEG,     t_final, 'linear', 'extrap');
ICurr_PV_i  = interp1(t2_corr, ICurr_PV,  t_final, 'linear', 'extrap');
V_PV_i      = interp1(t2_corr, V_PV,      t_final, 'linear', 'extrap');

%% 8. DATA EXPORT (.MAT)
% Save the processed data as a Timetable in a .mat file
fprintf('Preparing final dataset...\n');

% Generate dynamic filename based on the parent folder name
[~, folderName] = fileparts(strip(basePath, 'right', filesep)); 
outputFile = fullfile(basePath, sprintf('Global_%s.mat', folderName));

% Ensure time uniqueness for timetable creation
[t_unique, idx_unique] = unique(t_final);

% Aggregate data variables
data_vars = [ ...
    ICurr_TEG_i(idx_unique), ...     
    V_TEG_i(idx_unique), ...         
    ICurr_PV_i(idx_unique), ...      
    V_PV_i(idx_unique), ...          
    Vout_final(idx_unique), ...      
    Enable_final(idx_unique), ...    
    VbattT_final(idx_unique), ...    
    VbattP_final(idx_unique) ...     
];

variableNames = {'Vcurr_TEG', 'V_TEG', 'Vcurr_PV', 'V_PV', ...
                 'Vout', 'Enable', 'Vbatt_TEG', 'Vbatt_PV'};

% Create Timetable
% Note: seconds() converts the double vector to duration, required for Timetables
GlobalData = array2timetable(data_vars, ...
    'RowTimes', seconds(t_unique), ...
    'VariableNames', variableNames);

% Save to disk
% CRITICAL CHANGE: Added '-v7.3' to support files larger than 2GB (HDF5 format)
fprintf('Saving to disk (this may take time for large datasets)...\n');
save(outputFile, 'GlobalData', '-v7.3');

fprintf('------------------------------------------------\n');
fprintf('Export complete: %s\n', outputFile);
fprintf('Visualize data using: stackedplot(GlobalData)\n');
fprintf('------------------------------------------------\n');

% % --- VISUAL VERIFICATION ---
% figure('Name', 'Global Synchronization Verification');
% subplot(2,1,1);
% % Accessing RowTimes directly ensures compatibility
% plot(GlobalData.Time, GlobalData.Vout, 'b'); hold on;
% plot(GlobalData.Time, GlobalData.V_TEG, 'r--');
% title('Analog Signal Alignment'); legend('Vout (Ref)', 'V TEG (Sync)');
% grid on;
% 
% subplot(2,1,2);
% % Use original vectors for the check to verify the cut logic
% plot(t_final, D0_1(master_indices), 'k'); hold on;
% plot(t_final, interp1(t2_corr, D0_2, t_final, 'nearest'), 'm--');
% title('Digital Trigger Alignment'); legend('D0 Ref', 'D0 Sync');
% ylim([-0.2 5.2]); grid on;
%% --- HELPER FUNCTIONS ---

function t_out = fixTimeVector(t_in)
    % Stitches sawtooth time vectors (0..1, 0..1) into continuous (0..2)
    dt_diff = diff(t_in);
    jumps = find(dt_diff < 0);
    
    t_out = t_in;
    
    if ~isempty(jumps)
        current_offset = 0;
        for i = 1:length(jumps)
            idx = jumps(i);
            % Estimate step from previous sample
            dt_step = t_in(idx) - t_in(idx-1); 
            if isnan(dt_step) || dt_step <= 0, dt_step = 0.0001; end
            
            segment_end_time = t_in(idx);
            current_offset = current_offset + segment_end_time + dt_step;
            
            if i < length(jumps)
                next_idx = jumps(i+1);
                t_out(idx+1 : next_idx) = t_out(idx+1 : next_idx) + current_offset;
            else
                t_out(idx+1 : end) = t_out(idx+1 : end) + current_offset;
            end
        end
    end
end