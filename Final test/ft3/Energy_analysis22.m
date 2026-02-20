%% ENERGY HARVESTING SYSTEM ANALYSIS
% This script analyzes the power generation, storage, and consumption 
% of a hybrid energy harvesting system (TEG + PV).
%
% Methodology:
% MEASURED OFFESTS 
% 1. Sensor Calibration: Auto-zeroing based on dataset minimums.
% 2. Signal Conversion: Raw ADC voltage -> Current (using Shunt & Gain).
% 3. Energy Balance: 
%    - Input: Sum of positive increments in Total System Energy.
%    - Output: Sum of negative increments in Output Capacitor Energy.

clear; clc; close all;

% --- 1. CONFIGURATION AND DATA LOADING ---

basePath = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\Final test\ft3\ft3b2'; %CHANGE THE VALUE OF THE CAPACITOR!!!!
fileName = fullfile(basePath, 'Global_ft3b2.mat');

if ~isfile(fileName)
    error('Error: File not found at %s', fileName);
end

fprintf('Loading dataset: %s ...\n', fileName);
load(fileName, 'GlobalData');

% --- Hardware Constants ---
% Shunt Resistors (Ohm)
R_shunt_TEG = 0.2;
R_shunt_PV  = 10.0;

% Instrumentation Amplifier Gains (V/V)
Gain_TEG = 96.3;
Gain_PV  = 45.89;

%MAX GAIN (A/A) voltage current confersion with gain considered
GainA_PV = 459;
GainA_TEG = 21.6;

% Capacitor Bank Capacitances (Farad)
C_out = 44.25e-3;  % Main Output Buffer
C_bT  = 124.2e-3;  % TEG Internal Storage
C_bP  = 21.35e-3;   % PV Internal Storage

% Analysis Parameters
smooth_win = 3000; % Window size for smoothing energy derivatives | @15kS/s->3000 = 10NPLCs 10 periods of 50hertz)

% --- 2. SIGNAL PRE-PROCESSING & CALIBRATION ---
% A. Auto-Zero Calibration (Robust Mean of lowest 0.1%)
% We take the average of the lowest 0.1% of samples to define the noise floor.
n_total = length(GlobalData.Vcurr_TEG);
n_offset = max(1, round(0.001 * n_total)); % Ensure at least 1 sample

% Sort data to isolate the lowest values
sorted_TEG = sort(GlobalData.Vcurr_TEG);
sorted_PV  = sort(GlobalData.Vcurr_PV);

% Calculate average of the lowest n_offset samples
% offset_TEG = max(mean(sorted_TEG(1:n_offset)), 0);
% offset_PV  = max(mean(sorted_PV(1:n_offset)), 0);
% Measured offset
offset_TEG = 0.136;
offset_PV = 0.006;

% --- 2.B DATA SLICING (WHOLE CYCLES ONLY) ---
% Find the analysis window: from the first time Vout is FULL to the last time it is FULL.
% This ensures Energy_Final approx Energy_Initial for accurate efficiency calc.

n_total_original = length(GlobalData.Vcurr_TEG);
raw_Vout = GlobalData.Vout;

% 1. Determine "Full Charge" Threshold
% We consider "Max" as 99% of the absolute peak voltage found in the whole set.
v_max_abs = max(raw_Vout); 
v_threshold_high = v_max_abs * 0.99; 

% 2. Find indices where Vout is Full
high_indices = find(raw_Vout >= v_threshold_high);

if isempty(high_indices)
    warning('Voltage never reaches stable max. Using full dataset.');
else
    % 3. Define Start and End
    idx_start = high_indices(1);
    idx_end   = high_indices(end);
    
    % 4. Report Slicing Statistics
    n_discard_start = idx_start - 1;
    n_discard_end   = n_total_original - idx_end;
    
    fprintf('\n--- DATA SLICING REPORT ---\n');
    fprintf('Analysis Window defined by Vout max peaks (Threshold: %.3f V)\n', v_threshold_high);
    fprintf('Original Samples: %d\n', n_total_original);
    fprintf('Discarded Head:   %d (%.2f%%)\n', n_discard_start, (n_discard_start/n_total_original)*100);
    fprintf('Discarded Tail:   %d (%.2f%%)\n', n_discard_end, (n_discard_end/n_total_original)*100);
    fprintf('Kept Samples:     %d\n', idx_end - idx_start + 1);
    fprintf('---------------------------\n');
    
    % 5. Apply Slicing to GLOBALDATA
    GlobalData = GlobalData(idx_start:idx_end, :);
end

% B. Offset Removal and Clamping
% Subtract offset and clamp negative values to zero to remove noise floor artifacts.
V_adc_TEG_clean = GlobalData.Vcurr_TEG - offset_TEG;
V_adc_TEG_clean(V_adc_TEG_clean < 0) = 0;

V_adc_PV_clean  = GlobalData.Vcurr_PV - offset_PV;
V_adc_PV_clean(V_adc_PV_clean < 0) = 0;

% --- 3. CURRENT AND POWER CALCULATION ---

% % Calculate Real Current: I = V_adc / (Gain * R_shunt)
% I_TEG_V = V_adc_TEG_clean / (Gain_TEG * R_shunt_TEG);
% I_PV_V  = V_adc_PV_clean  / (Gain_PV  * R_shunt_PV);

% Calculate Real Current from characterisation model: 
% I = V_adc / (Gain[A/A]) gain and shunt considered
I_TEG = V_adc_TEG_clean / GainA_TEG;
I_PV  = V_adc_PV_clean  / GainA_PV;

% Calculate Instantaneous Power: P = V_source * I
P_TEG = GlobalData.V_TEG .* I_TEG;
P_PV  = GlobalData.V_PV  .* I_PV;

% Calculate Average Power (Mean over entire dataset)
Avg_P_TEG = mean(P_TEG);
Avg_P_PV  = mean(P_PV);

% --- 4. ENERGY FLUX ANALYSIS ---

% A. Total System Energy Calculation
% Sum of energy stored in all capacitors (Output + TEG Buffer + PV Buffer)
E_sys_raw = (0.5 * C_out * GlobalData.Vout.^2) + ...
            (0.5 * C_bT  * GlobalData.Vbatt_TEG.^2) + ...
            (0.5 * C_bP  * GlobalData.Vbatt_PV.^2);

% Apply smoothing to prevent noise from affecting derivative calculations
E_sys_smooth = smoothdata(E_sys_raw, 'gaussian', smooth_win);

% B. Net Energy Harvested (Input)
% Calculated by summing all POSITIVE increments of the Total System Energy.
% This captures energy entering from sources, regardless of which capacitor stores it.
dE_sys = [0; diff(E_sys_smooth)];
Energy_Harvested_Net = sum(dE_sys(dE_sys > 0));

% E. Storage State Correction (Residual Energy)
% Calculates the net change in stored energy between the start and end of the analysis.
% Since the analysis cuts based on Vout (Full->Full), the delta on Cout is negligible.
% However, C_bT and C_bP may have accumulated energy (Start=Empty, End=Full).
% This residual energy is "Harvested" but not "Consumed", so it must be credited.
E_sys_start = E_sys_smooth(1);
E_sys_end   = E_sys_smooth(end);
Delta_Storage_Residue = E_sys_end - E_sys_start;

% C. Energy Consumed (Output)
% Calculated by looking exclusively at the Output Capacitor (C_out).
% Sums all NEGATIVE increments (discharges), representing energy delivered to load.
E_out_smooth = smoothdata(0.5 * C_out * GlobalData.Vout.^2, 'gaussian', smooth_win);
dE_out = [0; diff(E_out_smooth)];
Energy_Consumed_Load = abs(sum(dE_out(dE_out < 0))) + Delta_Storage_Residue;

% D. Gross Generated Energy (Source Side)
% Integral of Power over time
time_sec = seconds(GlobalData.Time - GlobalData.Time(1));
E_Gen_TEG = trapz(time_sec, P_TEG);
E_Gen_PV  = trapz(time_sec, P_PV);
E_Gen_Total = E_Gen_TEG + E_Gen_PV;

% --- 5. RESULTS REPORT ---
fprintf('\n==================================================\n');
fprintf('           ENERGY ANALYSIS REPORT\n');
fprintf('==================================================\n');
fprintf('Calibration Offsets Removed:\n');
fprintf('   TEG Offset: %.6f V\n', offset_TEG);
fprintf('   PV Offset:  %.6f V\n', offset_PV);
%fprintf('   The offset was averaged using %d samples out of %d total.\n', n_offset, n_total);
fprintf('--------------------------------------------------\n');

fprintf('P/C rateo:\n');
fprintf('   TEG rateo: %.6f\n', Avg_P_TEG/C_bT);
fprintf('   PV  rateo: %.6f\n', Avg_P_PV/C_bP);
fprintf('--------------------------------------------------\n');

fprintf('1. POWER GENERATION (Average)\n');
fprintf('   TEG Avg Power:      %10.4f W  (%.2f mW)\n', Avg_P_TEG, Avg_P_TEG*1000);
fprintf('   PV Avg Power:       %10.4f W  (%.2f mW)\n', Avg_P_PV, Avg_P_PV*1000);
fprintf('\n');

fprintf('2. ENERGY BALANCE (Joules)\n');
fprintf('   [Generation Side]\n');
fprintf('   Total Source Energy:    %.4f J\n', E_Gen_Total);
fprintf('     - TEG Contribution:   %.4f J\n', E_Gen_TEG);
fprintf('     - PV Contribution:    %.4f J\n', E_Gen_PV);
fprintf('\n');
fprintf('   [Storage Side]\n');
fprintf('   Net Energy Stored:      %.4f J\n', Energy_Harvested_Net);
fprintf('   (Calculated from total system energy increments)\n');
fprintf('   Storage Efficiency:     %.2f %%\n', (Energy_Harvested_Net/E_Gen_Total)*100);
fprintf('\n');
fprintf('   [Load Side]\n');
fprintf('   Energy Delivered:       %.4f J\n', Energy_Consumed_Load);
fprintf('   (Calculated from Cout discharge cycles)\n');
fprintf('   Total Efficiency:     %.2f %%\n', (Energy_Consumed_Load/E_Gen_Total)*100);

fprintf('==================================================\n');

% --- 6. VISUALIZATION ---
figure('Name', 'System Energy Analysis', 'Color', 'w');

% Subplot 1: Source Power
subplot(3,1,1);
plot(GlobalData.Time, P_TEG*1000, 'r'); hold on;
plot(GlobalData.Time, P_PV*1000, 'b');
ylabel('Power (mW)'); 
title('Instantaneous Source Power (Calibrated)');
legend('TEG', 'PV'); grid on;

% Subplot 2: Capacitor Voltages
subplot(3,1,2);
plot(GlobalData.Time, GlobalData.Vout, 'k', 'LineWidth', 1.5); hold on;
plot(GlobalData.Time, GlobalData.Vbatt_TEG, 'r--');
plot(GlobalData.Time, GlobalData.Vbatt_PV, 'b--');
ylabel('Voltage (V)'); 
title('Capacitor Bank Voltages');
legend('Vout (Load)', 'Vbatt TEG', 'Vbatt PV'); grid on;

% Subplot 3: Energy Dynamics
subplot(3,1,3);
% Plot Total System Energy
plot(GlobalData.Time, E_sys_smooth, 'Color', [0, 0.6, 0], 'LineWidth', 1.5); hold on;
% Highlight Consumption phases (Cout discharge)
area(GlobalData.Time, -dE_out .* (dE_out < 0) * 100, 'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
ylabel('Energy (J)'); 
title('System Energy Storage (Green) vs Load Consumption Events (Red Area)');
legend('Total System Energy', 'Discharge Events'); 
grid on;