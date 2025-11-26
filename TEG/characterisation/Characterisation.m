%% ============================================================
%  TEG characterization analysis - ΔT ≈ 20°C
%  Date: 13/11/2025
%  Description:
%  Reads the Keithley CSV file and the thermal logger file,
%  synchronizes the data using teg_oc_voltage (mean -10%),
%  computes the average power per step and the mean/std temperature.
% =============================================================

clear; clc; close all;

%% === File paths ===
basePath = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\TEG\characterisation\DeltaT20\';
file_keithley = fullfile(basePath, '20.csv');
file_teg = fullfile(basePath, 'teg_data20.csv');

%% === Read Keithley data ===
opts = detectImportOptions(file_keithley);
Tk = readtable(file_keithley, opts);

% Reconstruct absolute time (in seconds)
disp(Tk.Time);
time_k = datetime(string(Tk.Time), 'InputFormat', 'HH:mm:ss');
time_k = seconds(time_k - time_k(1)); % relative time in seconds

I = abs(Tk.Reading);   % Current (A)
V = Tk.Value;          % Voltage (V)

%% === Read TEG temperature data ===
Tt = readtable(file_teg);

time_t = Tt.time / 1000; % time in seconds
Vteg = Tt.teg_oc_voltage;
T_delta = Tt.temp_delta;

%% === Synchronization "mean - 10%" ===
n0 = min(100, height(Tt));
mean_first = mean(Vteg(1:n0));
threshold = mean_first * 0.9;

idx_start = find(Vteg < threshold, 1, 'first');  % sweep start
if isempty(idx_start)
    error('Cannot find a point where teg_oc_voltage drops below mean -10%.');
end

% Sweep start time
t0 = time_t(idx_start);
t_end = time_k(end);

% Keep only temperature data during the sweep
mask = time_t >= t0 & time_t <= (t0 + t_end);
T_delta_valid = T_delta(mask);

T_mean = mean(T_delta_valid, 'omitnan');
T_std = std(T_delta_valid, 'omitnan');

fprintf('Average temperature difference: %.2f °C ± %.4f °C\n', T_mean, T_std);

%% === Compute power for each step ===
samples_per_step = 200;
n_steps = floor(length(V) / samples_per_step);

Vmean = zeros(n_steps,1);
Pmean = zeros(n_steps,1);
Pstd  = zeros(n_steps,1);

for k = 1:n_steps
    idx = (k-1)*samples_per_step + (1:samples_per_step);
    Vk = V(idx);
    Ik = I(idx);
    Pk = Vk .* Ik;

    Vmean(k) = mean(Vk);
    Pmean(k) = mean(Pk);
    Pstd(k)  = std(Pk);
end

%% === Figure 1: Mean power with error bars ===
figure('Name','Mean Power vs Voltage','Color','w');
errorbar(Vmean, Pmean, Pstd, 'o-', 'LineWidth',1.5, 'MarkerFaceColor','b');
xlabel('Voltage [V]');
ylabel('Mean Power [W]');
title(sprintf('TEG Characterization - ΔT ≈ %.1f°C ± %.1f°C', T_mean, T_std));
grid on;
legend(sprintf('T_{mean} = %.1f°C ± %.1f°C', T_mean, T_std), 'Location','best');


% %% === Optional: save results ===
% results = table(Vmean, Pmean, Pstd, repmat(T_mean,n_steps,1), repmat(T_std,n_steps,1), ...
%     'VariableNames',{'Vmean','Pmean','Pstd','Tmean','Tstd'});
% save(fullfile(basePath, 'results_TEG20.mat'), 'results');
% 
% disp('Analysis completed and plots generated successfully.');
