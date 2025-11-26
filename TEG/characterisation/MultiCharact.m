%% ============================================================
%  TEG characterization analysis for all available ΔT values
%  Produces a single plot of P(V) for each temperature
% =============================================================

clear; clc; close all;

%% === Base path containing all data folders ===
basePath = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\TEG\characterisation\';

% List of available temperature deltas
tempList = [5 10 15 20 25 30 35 40 45 50 55 65 75];%60 65 70 75 80 85 90];
legendEntries = cell(length(tempList), 1);

% Preallocate summary table
summary = table('Size',[length(tempList) 6], ...
                'VariableTypes',{'double','double','double','double','double','double'}, ...
                'VariableNames',{'DeltaT','ColdMean','HotMean','PowerMax','PowerMaxStd','AtVoltage'});

%% === Output figure ===
figure('Name','P-V Curves Comparison','Color','w');
hold on;
grid on;
xlabel('Average Voltage [V]');
ylabel('Average Power [W]');
title('TEG P(V) Curve for Each ΔT');

colors = lines(length(tempList));  % automatic color palette

%% ====== MAIN LOOP OVER ALL TEMPERATURES ======
for n = 1:length(tempList)

    Tval = tempList(n);
    
    fprintf("\n--- Processing ΔT = %d°C ---\n", Tval);

    % File paths
    folder = fullfile(basePath, 'Data');
    file_keithley = fullfile(folder, sprintf('%d.csv', Tval));
    file_teg      = fullfile(folder, sprintf('teg_data%d.csv', Tval));

    %% === Read Keithley sweep data ===
    opts = detectImportOptions(file_keithley);
    Tk = readtable(file_keithley, opts);

    % Time vector
    time_k = datetime(string(Tk.Time), 'InputFormat', 'HH:mm:ss');
    time_k = seconds(time_k - time_k(1));

    I = abs(Tk.Reading);   % current
    V = Tk.Value;          % voltage

    %% === Read TEG temperature data ===
    Tt = readtable(file_teg);

    time_t = Tt.time / 1000;
    Vteg   = Tt.teg_oc_voltage;
    Tdelta = Tt.temp_delta;

    Thot   = Tt.temp_hot_side;
    Tcold  = Tt.temp_cold_side;

    %% === Synchronization using "mean -10%" threshold ===
    n0 = min(100, height(Tt));
    mean_first = mean(Vteg(1:n0));
    threshold = mean_first * 0.9;

    idx_start = find(Vteg < threshold, 1, 'first');
    if isempty(idx_start)
        warning("ΔT = %d°C → could not detect drop below mean-10%%!", Tval);
        continue;
    end

    t0 = time_t(idx_start+10);  % start time + 10s first setting time (measure is: set V, delay 10s, 200 readings)
    t_end = time_k(end);

    % Valid temperature samples during sweep
    mask = time_t >= t0 & time_t <= (t0 + t_end);

    T_mean  = mean(Tdelta(mask), 'omitnan');
    T_std   = std(Tdelta(mask), 'omitnan');
    HotMean = mean(Thot(mask),   'omitnan');
    ColdMean= mean(Tcold(mask),  'omitnan');

%     %solving errors due to thermocouple offeset at 90°C
%     if Tval==90
%         T_mean=T_mean+16.5;
%         HotMean=HotMean+7;
%         ColdMean=ColdMean-9.5;
%     end

    fprintf("  Mean ΔT: %.2f ± %.3f°C\n", T_mean, T_std);

    %% === Compute average power for each step ===
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

    %% === Identify maximum power point ===
    [Pmax, idxMax] = max(Pmean);
    PmaxStd = Pstd(idxMax);
    V_at_Pmax = Vmean(idxMax);

    %% === Store values in summary table ===
    summary(n,:) = {T_mean, ColdMean, HotMean, Pmax, PmaxStd, V_at_Pmax};

    %% === Plot the curve for this ΔT ===
    errorbar(Vmean, Pmean, Pstd, ...
        '-', ...
        'Color', colors(n,:), ...
        'LineWidth', 1, ...
        'MarkerSize', 4, ...
        'MarkerFaceColor', 'none', ...
        'CapSize', 5 ...
    );

    legendEntries{n} = sprintf('ΔT = %.1f°C ± %.2f°C', T_mean, T_std);
end

%% === Final legend ===
legend(legendEntries, 'Location', 'best');
hold off;

disp("Processing completed for all temperature deltas!");

%% === Print summary table in console ===
disp(' ');
disp('=================== SUMMARY TABLE ===================');
disp(summary);
disp('======================================================');
