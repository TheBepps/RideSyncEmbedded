%% REAL PHYSICS ENERGY ANALYSIS (With Op-Amp Gains)
clear; clc; close all;

% --- 1. PARAMETRI HARDWARE ---
fileName = 'Global_ft2a.mat'; 
if ~isfile(fileName), error('File non trovato.'); end
load(fileName, 'GlobalData');

% Shunt Resistors (Ohm)
R_shunt_TEG = 0.2;
R_shunt_PV  = 10.0;

% Op-Amp Gains (V/V)
Gain_TEG = 113.0;
Gain_PV  = 46.0;

% Capacitors (Farad)
C_out = 240e-3; 
C_bT  = 120e-3; 
C_bP  = 33e-3;

% --- 2. PRE-PROCESSING SEGNALI (Offset) ---
% Rimuoviamo l'offset DC degli Op-Amp prima di dividere
% Assumiamo che il minimo valore assoluto letto sia lo zero fisico.

offset_TEG = min(GlobalData.Vcurr_TEG);
offset_PV  = 0.01; % Come da tua specifica (o min(GlobalData.Vcurr_PV))

% Applicazione Offset e pulizia negativi
V_adc_TEG_clean = GlobalData.Vcurr_TEG - offset_TEG;
V_adc_TEG_clean(V_adc_TEG_clean < 0) = 0;

V_adc_PV_clean  = GlobalData.Vcurr_PV - offset_PV;
V_adc_PV_clean(V_adc_PV_clean < 0) = 0;

% --- 3. CALCOLO CORRENTI REALI (Legge di Ohm + Gain) ---
% I = V_adc / (Gain * R_shunt)

I_TEG = V_adc_TEG_clean / (Gain_TEG * R_shunt_TEG);
I_PV  = V_adc_PV_clean  / (Gain_PV  * R_shunt_PV);

% --- 4. CALCOLO POTENZE ---
P_TEG = GlobalData.V_TEG .* I_TEG;
P_PV  = GlobalData.V_PV  .* I_PV;

% Medie
Avg_P_TEG = mean(P_TEG);
Avg_P_PV  = mean(P_PV);

% --- 5. CALCOLO ENERGIE (System Logic) ---
smooth_win = 20;

% A. Energia Totale del Sistema (Tutti i condensatori)
E_sys_raw = (0.5 * C_out * GlobalData.Vout.^2) + ...
            (0.5 * C_bT  * GlobalData.Vbatt_sec_TEG.^2) + ...
            (0.5 * C_bP  * GlobalData.Vbatt_sec_PV.^2);

E_sys_smooth = smoothdata(E_sys_raw, 'gaussian', smooth_win);

% B. Input: Somma degli incrementi netti del sistema
dE_sys = [0; diff(E_sys_smooth)];
Energy_Harvested_Net = sum(dE_sys(dE_sys > 0));

% C. Output: Somma delle scariche di Cout (Carico)
E_out_smooth = smoothdata(0.5 * C_out * GlobalData.Vout.^2, 'gaussian', smooth_win);
dE_out = [0; diff(E_out_smooth)];
Energy_Consumed = abs(sum(dE_out(dE_out < 0)));

% D. Input alle Sorgenti (Generata Lorda)
E_Source_Generated = trapz(seconds(GlobalData.Time), P_TEG + P_PV);

% --- 6. REPORT ---
fprintf('\n==================================================\n');
fprintf('       REPORT DATI REALI (Gain Corrected)\n');
fprintf('==================================================\n');
fprintf('Gain TEG applicato: %.1fx\n', Gain_TEG);
fprintf('Gain PV  applicato: %.1fx\n', Gain_PV);
fprintf('--------------------------------------------------\n');

fprintf('1. POTENZA MEDIA (Target: ~200mW / ~4mW)\n');
fprintf('   TEG Avg Power:      %10.4f W  (%.2f mW)\n', Avg_P_TEG, Avg_P_TEG*1000);
fprintf('   PV Avg Power:       %10.4f W  (%.2f mW)\n', Avg_P_PV, Avg_P_PV*1000);
fprintf('\n');

fprintf('2. BILANCIO ENERGETICO (Joule)\n');
fprintf('   E_Generated (Sources):  %.4f J\n', E_Source_Generated);
fprintf('   E_Stored (Net Input):   %.4f J (Efficienza: %.1f%%)\n', ...
    Energy_Harvested_Net, (Energy_Harvested_Net/E_Source_Generated)*100);
fprintf('   E_Consumed (Load):      %.4f J\n', Energy_Consumed);

fprintf('\n3. VERIFICA VOLTAGGI OP-AMP (Max letti)\n');
fprintf('   Max V_ADC TEG: %.3f V -> I_max reale: %.3f A\n', ...
    max(GlobalData.Vcurr_TEG), max(I_TEG));
fprintf('   Max V_ADC PV:  %.3f V -> I_max reale: %.3f A\n', ...
    max(GlobalData.Vcurr_PV), max(I_PV));

fprintf('==================================================\n');

% --- 7. PLOT ---
figure('Name', 'Analisi Reale con Gain');
subplot(2,1,1);
plot(GlobalData.Time, P_TEG*1000, 'r'); hold on;
plot(GlobalData.Time, P_PV*1000, 'b');
ylabel('Potenza (mW)'); title('Potenza Reale (Gain Corrected)');
legend('TEG', 'PV'); grid on;

subplot(2,1,2);
plot(GlobalData.Time, E_sys_smooth, 'k');
ylabel('Joule'); title('Energia Totale Sistema Accumulata');
grid on;