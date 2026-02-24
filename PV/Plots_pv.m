%% RideSync: Charge-Sharing Efficiency Plotter (Single Y-Axis)
% Data processing for ft6 test series
% Channels: A = Enable, B = Vin, C = Vout

clear; close all; clc;

% Setup directory and file pattern
data_dir = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\PV\Plots';
%file_list = dir(fullfile(data_dir, '20251210-SummaryTest_EN-Vteg-Vpv-Vout_PVdiretto.mat'));
%file_list = dir(fullfile(data_dir, '20251211_Vpv_con solo condensatore_3 colpi.mat'));
file_list = dir(fullfile(data_dir, '20251211_Vpv-Vout-Vcond_BQ_3 colpi.mat'));

if isempty(file_list)
    error('No .mat files found in the target directory.');
end

% Define custom titles for the three expected scenarios
custom_titles = {
    %'Initial direct-coupling setup'%, 
    %'Pulsed light test on direct connection', 
    'Performance validation with BQ25505'
};

for i = 1:length(file_list)
    file_path = fullfile(data_dir, file_list(i).name);
    raw = load(file_path);
    
    % Extract signal arrays
    v_in    = double(raw.A);
    v_Csto  = double(raw.B);
    v_out = double(raw.C);
    %v_out = double(raw.D);
    
    % Extract timing metadata
    t_start    = double(raw.Tstart);
    t_interval = double(raw.Tinterval);
    
    % Derive length directly from the data array
    num_samples = length(v_in);
    
    % Construct time vector
    t = t_start + (0:num_samples-1) * t_interval;
    
    % Plot setup
    figure('Name', file_list(i).name, 'Color', 'w');
    
    % Single Axis Plotting
    plot(t, v_in, 'b-', 'LineWidth', 1.5, 'DisplayName', 'V_{PV}');
    hold on;
    plot(t, v_Csto, 'r-', 'LineWidth', 1.5, 'DisplayName', 'V_{Csto}');
    plot(t, v_out, '-', 'Color', [0 0.5 0], 'LineWidth', 1.5, 'DisplayName', 'V_{out}');
    %plot(t, v_out, '-', 'Color','yellow', 'LineWidth', 1.5, 'DisplayName', 'V_{out}');
    
    % Axis formatting
    ylabel('Voltage [V]', 'Color', 'k');
    ylim([-0.5 5.5]);
    grid on;
    
    ylim([0 5]);
    xlim([-0.5 4.5]);
    
    % Assign title based on iteration index
    if i <= length(custom_titles)
        title(custom_titles{i}, 'Color', 'k');
    else
        % Fallback if there are more than 3 files
        title(['Transfer Transient: ', strrep(file_list(i).name, '_', ' ')], 'Color', 'k');
    end
    
    xlabel('Time [s]', 'Color', 'k');
    legend('Location', 'southoutside', 'Orientation', 'horizontal', 'TextColor', 'k');
    
    ax = gca;
    ax.XColor = 'k';
    ax.YColor = 'k'; 
end