%% === Confronto di rumore e spettri di due configurazioni MAX40018 ===
% Overlay, media ± std, confronto tra configurazioni
% Data: 2025-10-09

clear; clc; close all;

%% === Cartelle delle due configurazioni ===
folder1 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251009-analisi_rumore-400k-Vin_Vout1_V+_Vout2';
folder2 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251009-analisi_rumore-4meg-Vin_Vout1_V+_Vout2';
folder3 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251014-analisi_rumore_DC-400k-Vin_Vout1_V+_Vout2';
folder4 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251014-analisi_rumore_DC-4Meg-Vin_Vout1_V+_Vout2';

%% === Parametri di integrazione ===
fmin = 0.04;   % kHz -> 40 Hz
fmax = 10;     % kHz -> 10 kHz

%% === Elaborazione configurazioni ===
[f1, B1, D1, A1, C1, meanB1, meanD1, meanA1, meanC1, stdB1, stdD1, stdA1, stdC1, nB1, nD1] = processFolder(folder1, fmin, fmax);
[f2, B2, D2, A2, C2, meanB2, meanD2, meanA2, meanC2, stdB2, stdD2, stdA2, stdC2, nB2, nD2] = processFolder(folder2, fmin, fmax);
% Load DC-powered versions
[f3, B3, D3, A3, C3, meanB3, meanD3, meanA3, meanC3, stdB3, stdD3, stdA3, stdC3, nB3, nD3] = processFolder(folder3, fmin, fmax);
[f4, B4, D4, A4, C4, meanB4, meanD4, meanA4, meanC4, stdB4, stdD4, stdA4, stdC4, nB4, nD4] = processFolder(folder4, fmin, fmax);

%% === Overlay for each configuration (4 setups) ===
for cfg = 1:4
    figure('Name', sprintf('Overlay Config %d', cfg), 'NumberTitle', 'off'); hold on;

    switch cfg
        case 1
            allA = A1; allB = B1; allC = C1; allD = D1; 
            meanA_plt = meanA1; meanB_plt = meanB1; meanC_plt = meanC1; meanD_plt = meanD1;
            stdA_plt = stdA1; stdB_plt = stdB1; stdC_plt = stdC1; stdD_plt = stdD1;
            f = f1; label = '400k (Standard)';
            nB = nB1; nD = nD1;
        case 2
            allA = A2; allB = B2; allC = C2; allD = D2;
            meanA_plt = meanA2; meanB_plt = meanB2; meanC_plt = meanC2; meanD_plt = meanD2;
            stdA_plt = stdA2; stdB_plt = stdB2; stdC_plt = stdC2; stdD_plt = stdD2;
            f = f2; label = '4M (Standard)';
            nB = nB2; nD = nD2;
        case 3
            allA = A3; allB = B3; allC = C3; allD = D3;
            meanA_plt = meanA3; meanB_plt = meanB3; meanC_plt = meanC3; meanD_plt = meanD3;
            stdA_plt = stdA3; stdB_plt = stdB3; stdC_plt = stdC3; stdD_plt = stdD3;
            f = f3; label = '400k (DC supply)';
            nB = nB3; nD = nD3;
        case 4
            allA = A4; allB = B4; allC = C4; allD = D4;
            meanA_plt = meanA4; meanB_plt = meanB4; meanC_plt = meanC4; meanD_plt = meanD4;
            stdA_plt = stdA4; stdB_plt = stdB4; stdC_plt = stdC4; stdD_plt = stdD4;
            f = f4; label = '4M (DC supply)';
            nB = nB4; nD = nD4;
    end

    % Transparent spectra
    plot(f, allA, 'Color', [0.5 0.5 0.5]);
    plot(f, allB, 'Color', [0.7 0.85 1]);
    plot(f, allC, 'Color', [1 1 0.6]);
    plot(f, allD, 'Color', [1 0.8 0.8]);

    % ±σ patches
    fillX = [f; flipud(f)];
    fillY_A = [meanA_plt + stdA_plt; flipud(meanA_plt - stdA_plt)];
    fillY_B = [meanB_plt + stdB_plt; flipud(meanB_plt - stdB_plt)];
    fillY_C = [meanC_plt + stdC_plt; flipud(meanC_plt - stdC_plt)];
    fillY_D = [meanD_plt + stdD_plt; flipud(meanD_plt - stdD_plt)];

    patch('XData', fillX, 'YData', fillY_C, 'FaceColor', 'y', 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    patch('XData', fillX, 'YData', fillY_A, 'FaceColor', [0.6 0.6 0.6], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    patch('XData', fillX, 'YData', fillY_B, 'FaceColor', 'b', 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    patch('XData', fillX, 'YData', fillY_D, 'FaceColor', 'r', 'FaceAlpha', 0.12, 'EdgeColor', 'none');

    % Mean lines
    hC = plot(f, meanC_plt, 'y-', 'LineWidth', 1.5); % V+
    hA = plot(f, meanA_plt, 'k-', 'LineWidth', 1.5); % Vin
    hB = plot(f, meanB_plt, 'b-', 'LineWidth', 1.5); % Vout1    
    hD = plot(f, meanD_plt, 'r-', 'LineWidth', 1.5); % Vout2

    xlim([0 1]);
    ylim([-100 20]);
    xlabel('Frequency [kHz]');
    ylabel('Amplitude [dBu]');
    title({['Overlay ', label, ': Vin(A), Vout1(B), V+(C), Vout2(D)'], ...
           sprintf('Integrated Noise [µV RMS]: Vout1 = %.2f, Vout2 = %.2f', mean(nB), mean(nD))});
    legend([hA hB hC hD], {'Vin', 'Vout1', 'V+', 'Vout2'}, 'Location', 'best');
end


%% === Comparison: DC vs Standard Supply ===
% === Channel D plot ===
figure('Name','Vout2_4MOhm - Batteries vs Power Supply','NumberTitle','off');

% --- 4 MOhm ---
hold on;
fillX = [f2; flipud(f2)];
fillY_std = [meanD2+stdD2; flipud(meanD2-stdD2)];
fillY_dc  = [meanD4+stdD4; flipud(meanD4-stdD4)];
patch('XData',fillX,'YData',fillY_std,'FaceColor',[1 0 0],'FaceAlpha',0.12,'EdgeColor','none');
patch('XData',fillX,'YData',fillY_dc, 'FaceColor',[0 0.5 1],'FaceAlpha',0.12,'EdgeColor','none');
h3 = plot(f2,meanD2,'Color',[1 0 0],'LineWidth',1.5);
h4 = plot(f2,meanD4,'Color',[0 0.5 1],'LineWidth',1.5);
xlabel('Frequency [kHz]'); ylabel('Amplitude [dBu]');
title('Channel D - 4MOhm: Batteries vs Power Supply');
legend([h3 h4],{'Vout2 - Power supply','Vout2 - DC (batteries)'},'Location','best');
xlim([0 1]);
ylim([-100 20]); grid on;

%% === DC batteries both resistance configurations ===
figure('Name','Batteries Powered - Channel D - 400kOhm vs 4MOhm','NumberTitle','off'); hold on;

% --- 400 kOhm ---
fillX1 = [f1; flipud(f1)];
fillY_dc1  = [meanD3+stdD3; flipud(meanD3-stdD3)];
patch('XData',fillX1,'YData',fillY_dc1, 'FaceColor',[1 0 0],'FaceAlpha',0.12,'EdgeColor','none');
h2 = plot(f1,meanD3,'Color',[1 0 0],'LineWidth',1.5);

% --- 4 MOhm ---
fillX2 = [f2; flipud(f2)];
fillY_dc2  = [meanD4+stdD4; flipud(meanD4-stdD4)];
patch('XData',fillX2,'YData',fillY_dc2, 'FaceColor',[0 0.5 1],'FaceAlpha',0.12,'EdgeColor','none');
h4 = plot(f2,meanD4,'Color',[0 0.5 1],'LineWidth',1.5);

xlabel('Frequency [kHz]');
ylabel('Amplitude [dBu]');
title('Channel D - Comparison: 400kOhm and 4MOhm, Batteries supply');
legend([ h2  h4],{'Vout2 - 400kOhm config','Vout2 - 4MOhm config'},'Location','best');
xlim([0 1]);
ylim([-100 20]);
grid on;


%% === Confronto dei rumori integrati ===
disp('=== Confronto rumore integrato (40 Hz – 10 kHz) ===');
gainB_alim_dB = 20*log10(mean(nB2)/mean(nB1));
gainD_alim_dB = 20*log10(mean(nD2)/mean(nD1));
gainB_batt_dB = 20*log10(mean(nB4)/mean(nB3));
gainD_batt_dB = 20*log10(mean(nD4)/mean(nD3));
fprintf('Channel B: Config2 rispetto a Config1 power supply = %.2f dB di differenza\n', gainB_alim_dB);
fprintf('Channel D: Config2 rispetto a Config1 power supply = %.2f dB di differenza\n', gainD_alim_dB);
fprintf('Channel B: Config2 rispetto a Config1 batteries = %.2f dB di differenza\n', gainB_batt_dB);
fprintf('Channel D: Config2 rispetto a Config1 batteries = %.2f dB di differenza\n', gainD_batt_dB);

%% === Funzione locale aggiornata ===
function [freq, allB, allD, allA, allC, meanB, meanD, meanA, meanC, stdB, stdD, stdA, stdC, nB_uV, nD_uV] = processFolder(folder, fmin, fmax)
    files_struct = dir(fullfile(folder, '*.csv'));
    files = {files_struct.name};
    if isempty(files)
        error(['Nessun file CSV in ', folder]);
    end
    numFiles = numel(files);

    for k = 1:numFiles
        fname = fullfile(folder, files{k});
        opts = detectImportOptions(fname, 'NumHeaderLines', 2);
        opts.VariableNamesLine = 2;
        T = readtable(fname, opts);
        freq_kHz = T{:,1};
        chA_dBu = T{:,2}; % Vin
        chB_dBu = T{:,3}; % Vout1
        chC_dBu = T{:,4}; % V+
        chD_dBu = T{:,5}; % Vout2

        if k == 1
            freq = freq_kHz;
            allA = zeros(numel(freq), numFiles);
            allB = zeros(numel(freq), numFiles);
            allC = zeros(numel(freq), numFiles);
            allD = zeros(numel(freq), numFiles);
        end

        allA(:,k) = chA_dBu;
        allB(:,k) = chB_dBu;
        allC(:,k) = chC_dBu;
        allD(:,k) = chD_dBu;

        % Conversione in Vrms
        Vrms_B = 0.775 * 10.^(chB_dBu/20);
        Vrms_D = 0.775 * 10.^(chD_dBu/20);

        df_Hz = diff(freq_kHz)*1e3; df_Hz(end+1) = df_Hz(end);
        mask = (freq_kHz >= fmin) & (freq_kHz <= fmax);

        nB_uV(k) = sqrt(sum((Vrms_B(mask)).^2 .* df_Hz(mask))) * 1e6;
        nD_uV(k) = sqrt(sum((Vrms_D(mask)).^2 .* df_Hz(mask))) * 1e6;
    end

    % Medie e deviazioni standard
    meanA = mean(allA, 2); stdA = std(allA, 0, 2);
    meanB = mean(allB, 2); stdB = std(allB, 0, 2);
    meanC = mean(allC, 2); stdC = std(allC, 0, 2);
    meanD = mean(allD, 2); stdD = std(allD, 0, 2);

    disp(['--- ', folder, ' ---']);
    disp(table(files', nB_uV', nD_uV', 'VariableNames', {'File','NoiseB_uVrms','NoiseD_uVrms'}));
    disp(['Media rumore B: ', num2str(mean(nB_uV),'%.2f'), ' µV RMS']);
    disp(['Media rumore D: ', num2str(mean(nD_uV),'%.2f'), ' µV RMS']);
    disp(['Deviazione std rumore B: ', num2str(std(nB_uV),'%.2f'), ' µV RMS']);
    disp(['Deviazione std rumore D: ', num2str(std(nD_uV),'%.2f'), ' µV RMS']);
end
