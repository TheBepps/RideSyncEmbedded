%% === Analisi integrale e overlay di spettri OpAmp MAX40018 ===
% Mostra 10 spettri in trasparenza, media e deviazione std
% Calcola anche il rumore integrato (µV RMS)
% Data: 2025-10-09

clear; clc; close all;

% === Cartella contenente i file CSV ===
folder = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251008-analisi_rumore-400k';

files_struct = dir(fullfile(folder, '*.csv'));
files = {files_struct.name};

if isempty(files)
    error('Nessun file CSV trovato nella cartella specificata.');
end

% === Parametri di integrazione ===
fmin = 0.05;   % kHz -> 50 Hz
fmax = 10;    % kHz -> 10 kHz

% === Preallocazione ===
numFiles = numel(files);
allFreq = [];
allB = [];
allD = [];
noise_rms_uV_B = zeros(numFiles,1);
noise_rms_uV_D = zeros(numFiles,1);

for k = 1:numFiles
    fname = fullfile(folder, files{k});
    
    % Lettura con doppia intestazione
    opts = detectImportOptions(fname, 'NumHeaderLines', 2);
    opts.VariableNamesLine = 2;
    T = readtable(fname, opts);

    freq_kHz = T{:,1};
    chB_dBu  = T{:,2};
    chD_dBu  = T{:,3};

    if isempty(allFreq)
        allFreq = freq_kHz;
        allB = zeros(numel(freq_kHz), numFiles);
        allD = zeros(numel(freq_kHz), numFiles);
    end

    allB(:,k) = chB_dBu;
    allD(:,k) = chD_dBu;

    % Conversione in Vrms
    Vrms_B = 0.775 * 10.^(chB_dBu/20);
    Vrms_D = 0.775 * 10.^(chD_dBu/20);

    df_Hz = diff(freq_kHz)*1e3; df_Hz(end+1) = df_Hz(end);
    mask = (freq_kHz >= fmin) & (freq_kHz <= fmax);

    % Rumore integrato (µV RMS)
    noise_rms_uV_B(k) = sqrt(sum((Vrms_B(mask)).^2 .* df_Hz(mask))) * 1e6;
    noise_rms_uV_D(k) = sqrt(sum((Vrms_D(mask)).^2 .* df_Hz(mask))) * 1e6;
end

% === Calcoli statistici ===
meanB = mean(allB, 2);
meanD = mean(allD, 2);
stdB = std(allB, 0, 2);
stdD = std(allD, 0, 2);

% === Overlay Plot ===
fh = figure('Name','Overlay Spettri OpAmp','NumberTitle','off');
hold on;

% Tutti gli spettri in trasparenza
plot(allFreq, allB, 'Color', [0.7 0.85 1]); 
plot(allFreq, allD, 'Color', [1 0.8 0.8]);

% Patch ±1σ (deviazione standard)
fillX = [allFreq; flipud(allFreq)];
fillY_B = [meanB+stdB; flipud(meanB-stdB)];
fillY_D = [meanD+stdD; flipud(meanD-stdD)];
patch('XData', fillX, 'YData', fillY_B, 'FaceColor', 'b', 'FaceAlpha', 0.15, 'EdgeColor','none');
patch('XData', fillX, 'YData', fillY_D, 'FaceColor', 'r', 'FaceAlpha', 0.12, 'EdgeColor','none');

% Linee medie
hB = plot(allFreq, meanB, 'b-', 'LineWidth', 1);
hD = plot(allFreq, meanD, 'r-', 'LineWidth', 1);

xlabel('Frequenza [kHz]');
ylabel('Ampiezza [dBu]');
title('Overlay: Channel B (blu) e Channel D (rosso)');
legend([hB hD], {'Media Channel B','Media Channel D'}, 'Location','best');
%set(gca,'XScale','log'); grid on;

% === Risultati numerici ===
disp('--- Rumore integrato (50 Hz – 10 kHz) ---');
T = table(files', noise_rms_uV_B, noise_rms_uV_D, ...
    'VariableNames', {'File','NoiseB_uVrms','NoiseD_uVrms'});
disp(T);
disp('--- Statistiche ---');
disp(['Media rumore B: ', num2str(mean(noise_rms_uV_B),'%.2f'), ' µV RMS']);
disp(['Media rumore D: ', num2str(mean(noise_rms_uV_D),'%.2f'), ' µV RMS']);
disp(['Dev std rumore B: ', num2str(std(noise_rms_uV_B),'%.2f'), ' µV RMS']);
disp(['Dev std rumore D: ', num2str(std(noise_rms_uV_D),'%.2f'), ' µV RMS']);
