%% === Confronto di rumore e spettri di due configurazioni MAX40018 ===
% Overlay, media ± std, confronto tra configurazioni
% Data: 2025-10-15

clear; clc; close all;

%% === Cartelle delle due configurazioni ===
folder1 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251014-analisi_rumore_V2-400k-Vin_Vout1_V+_Vout2';
folder2 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251014-analisi_rumore_V2-4meg-Vin_Vout1_V+_Vout2';
folder3 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251014-analisi_rumore_DC_V2-400k-Vin_Vout1_V+_Vout2';
folder4 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251014-analisi_rumore_DC_V2-4Meg-Vin_Vout1_V+_Vout2';

%% === Parametri di integrazione ===
fmin = 0.04;   % kHz -> 40 Hz
fmax = 2;     % kHz -> 2 kHz

%% === Elaborazione configurazioni ===
[f1, B1, D1, A1, C1, meanB1, meanD1, meanA1, meanC1, stdB1, stdD1, stdA1, stdC1, nB1, nD1] = processFolder(folder1, fmin, fmax);
[f2, B2, D2, A2, C2, meanB2, meanD2, meanA2, meanC2, stdB2, stdD2, stdA2, stdC2, nB2, nD2] = processFolder(folder2, fmin, fmax);
% Load DC-powered versions
[f3, B3, D3, A3, C3, meanB3, meanD3, meanA3, meanC3, stdB3, stdD3, stdA3, stdC3, nB3, nD3] = processFolder(folder3, fmin, fmax);
[f4, B4, D4, A4, C4, meanB4, meanD4, meanA4, meanC4, stdB4, stdD4, stdA4, stdC4, nB4, nD4] = processFolder(folder4, fmin, fmax);

%% === Overlay for each configuration (FAST MODE) ===

fastPlot = true;     % toggle: velocissimo, false = alta qualità
maxPts   = 25000;    % Numero max punti da visualizzare per canale

for cfg = 1:4
    figure('Name', sprintf('Overlay Config %d', cfg), 'NumberTitle', 'off'); hold on;
    set(gcf, 'Renderer', 'painters');  % più veloce per 2D

    % --- Selezione dati per configurazione ---
    switch cfg
        case 1
            allA = A1; allB = B1; allC = C1; allD = D1; 
            meanA_plt = meanA1; meanB_plt = meanB1; meanC_plt = meanC1; meanD_plt = meanD1;
            stdA_plt = stdA1; stdB_plt = stdB1; stdC_plt = stdC1; stdD_plt = stdD1;
            f = f1; label = '400k (Power supply)'; nB = nB1; nD = nD1;
        case 2
            allA = A2; allB = B2; allC = C2; allD = D2;
            meanA_plt = meanA2; meanB_plt = meanB2; meanC_plt = meanC2; meanD_plt = meanD2;
            stdA_plt = stdA2; stdB_plt = stdB2; stdC_plt = stdC2; stdD_plt = stdD2;
            f = f2; label = '4M (Power supply)'; nB = nB2; nD = nD2;
        case 3
            allA = A3; allB = B3; allC = C3; allD = D3;
            meanA_plt = meanA3; meanB_plt = meanB3; meanC_plt = meanC3; meanD_plt = meanD3;
            stdA_plt = stdA3; stdB_plt = stdB3; stdC_plt = stdC3; stdD_plt = stdD3;
            f = f3; label = '400k (Batteries supply)'; nB = nB3; nD = nD3;
        case 4
            allA = A4; allB = B4; allC = C4; allD = D4;
            meanA_plt = meanA4; meanB_plt = meanB4; meanC_plt = meanC4; meanD_plt = meanD4;
            stdA_plt = stdA4; stdB_plt = stdB4; stdC_plt = stdC4; stdD_plt = stdD4;
            f = f4; label = '4M (Batteries supply)'; nB = nB4; nD = nD4;
    end

    % --- Decimate per ridurre i punti ---
    if numel(f) > maxPts && fastPlot
        q = ceil(numel(f)/maxPts);  % fattore di decimazione
        f_plot = decimate(f, q, 'fir');  % decimate frequenze
        meanA_plot = decimate(meanA_plt, q, 'fir');
        meanB_plot = decimate(meanB_plt, q, 'fir');
        meanC_plot = decimate(meanC_plt, q, 'fir');
        meanD_plot = decimate(meanD_plt, q, 'fir');
        stdA_plot  = decimate(stdA_plt, q, 'fir');
        stdB_plot  = decimate(stdB_plt, q, 'fir');
        stdC_plot  = decimate(stdC_plt, q, 'fir');
        stdD_plot  = decimate(stdD_plt, q, 'fir');
    else
        f_plot = f;
        meanA_plot = meanA_plt;
        meanB_plot = meanB_plt;
        meanC_plot = meanC_plt;
        meanD_plot = meanD_plt;
        stdA_plot  = stdA_plt;
        stdB_plot  = stdB_plt;
        stdC_plot  = stdC_plt;
        stdD_plot  = stdD_plt;
    end

    % --- Tracciare poche trace originali selezionate (trasparenza) ---
    if fastPlot
        nLines = min(5, size(allB, 2));
        sel = round(linspace(1, size(allB, 2), nLines));
        % decimare anche i raw per far combaciare le dimensioni
        for k = sel
            plot(decimate(f, q, 'fir'), decimate(allA(:,k), q, 'fir'), 'Color', [0.5 0.5 0.5]);
            plot(decimate(f, q, 'fir'), decimate(allB(:,k), q, 'fir'), 'Color', [0.7 0.85 1]);
            plot(decimate(f, q, 'fir'), decimate(allC(:,k), q, 'fir'), 'Color', [1 1 0.6]);
            plot(decimate(f, q, 'fir'), decimate(allD(:,k), q, 'fir'), 'Color', [1 0.8 0.8]);
        end
    end

    % --- ±σ area ---
    patch('XData',[f_plot; flipud(f_plot)], 'YData',[meanB_plot+stdB_plot; flipud(meanB_plot-stdB_plot)], ...
          'FaceColor','b','FaceAlpha',0.12,'EdgeColor','none');
    patch('XData',[f_plot; flipud(f_plot)], 'YData',[meanC_plot+stdC_plot; flipud(meanC_plot-stdC_plot)], ...
          'FaceColor','y','FaceAlpha',0.12,'EdgeColor','none');
    patch('XData',[f_plot; flipud(f_plot)], 'YData',[meanA_plot+stdA_plot; flipud(meanA_plot-stdA_plot)], ...
          'FaceColor',[0.6 0.6 0.6],'FaceAlpha',0.12,'EdgeColor','none');
    patch('XData',[f_plot; flipud(f_plot)], 'YData',[meanD_plot+stdD_plot; flipud(meanD_plot-stdD_plot)], ...
          'FaceColor','r','FaceAlpha',0.12,'EdgeColor','none');

    % --- Linee medie ---
    hA = plot(f_plot, meanA_plot, 'k-', 'LineWidth', 1);
    hB = plot(f_plot, meanB_plot, 'b-', 'LineWidth', 1);
    hC = plot(f_plot, meanC_plot, 'y-', 'LineWidth', 1);
    hD = plot(f_plot, meanD_plot, 'r-', 'LineWidth', 1);

    % --- Etichette e limiti ---
    xlim([0 1]); ylim([-130 20]);
    xlabel('Frequency [kHz]'); ylabel('Amplitude [dBu]');
    title({['Overlay ', label, ': Vin(A), Vout1(B), V+(C), Vout2(D)'], ...
           sprintf('Integrated Noise(%.2f-%.2f kHz) [µV RMS]: Vout1 = %.2f, Vout2 = %.2f', fmin, fmax, mean(nB), mean(nD))});
    legend([hA hB hC hD], {'Vin','Vout1','V+','Vout2'}, 'Location','best');

    grid on;
    drawnow limitrate; % riduce ridisegni multipli
end


%% === Comparison: DC vs Standard Supply ===
figure('Name','Vout2_4MOhm - Batteries vs Power Supply','NumberTitle','off');
hold on;
set(gcf,'Renderer','painters');  % più veloce per 2D

% --- Downsample con decimate ---
maxPts = 26214;  % compromesso qualità/velocità
if numel(f2) > maxPts
    q2 = ceil(numel(f2)/maxPts);
    f2d      = decimate(f2, q2, 'fir');
    meanD2d  = decimate(meanD2, q2, 'fir');
    stdD2d   = decimate(stdD2, q2, 'fir');
    meanD4d  = decimate(meanD4, q2, 'fir');
    stdD4d   = decimate(stdD4, q2, 'fir');
else
    f2d      = f2;
    meanD2d  = meanD2; stdD2d = stdD2;
    meanD4d  = meanD4; stdD4d = stdD4;
end

% --- Patch con meno vertici ---
patch('XData',[f2d; flipud(f2d)], 'YData',[meanD2d+stdD2d; flipud(meanD2d-stdD2d)], ...
      'FaceColor',[1 0 0],'FaceAlpha',0.12,'EdgeColor','none');
patch('XData',[f2d; flipud(f2d)], 'YData',[meanD4d+stdD4d; flipud(meanD4d-stdD4d)], ...
      'FaceColor',[0 0.5 1],'FaceAlpha',0.12,'EdgeColor','none');

% --- Linee medie ---
h3 = plot(f2d, meanD2d, 'Color',[1 0 0],'LineWidth',1);
h4 = plot(f2d, meanD4d, 'Color',[0 0.5 1],'LineWidth',1);

xlabel('Frequency [kHz]'); ylabel('Amplitude [dBu]');
title('Channel D - 4MΩ: Batteries vs Power Supply');
legend([h3 h4],{'Vout2 - Power supply','Vout2 - DC (batteries)'},'Location','best');
xlim([0 2]); ylim([-100 20]); grid on;
drawnow limitrate;

%% === DC batteries both resistance configurations ===
figure('Name','Batteries Powered - Channel D - 400kΩ vs 4MΩ','NumberTitle','off');
hold on;
set(gcf,'Renderer','painters');

% --- Downsample con decimate ---
if numel(f1) > maxPts
    q1 = ceil(numel(f1)/maxPts);
    f1d      = decimate(f1, q1, 'fir');
    meanD3d  = decimate(meanD3, q1, 'fir');
    stdD3d   = decimate(stdD3, q1, 'fir');
else
    f1d      = f1;
    meanD3d  = meanD3; stdD3d = stdD3;
end

% Riutilizziamo f2d e meanD4d/ stdD4d da sopra, decimati già
% --- Patch ---
patch('XData',[f1d; flipud(f1d)], 'YData',[meanD3d+stdD3d; flipud(meanD3d-stdD3d)], ...
      'FaceColor',[1 0 0],'FaceAlpha',0.12,'EdgeColor','none');
patch('XData',[f2d; flipud(f2d)], 'YData',[meanD4d+stdD4d; flipud(meanD4d-stdD4d)], ...
      'FaceColor',[0 0.5 1],'FaceAlpha',0.12,'EdgeColor','none');

% --- Linee medie ---
h2 = plot(f1d, meanD3d, 'Color',[1 0 0],'LineWidth',1);
h4 = plot(f2d, meanD4d, 'Color',[0 0.5 1],'LineWidth',1);

xlabel('Frequency [kHz]'); ylabel('Amplitude [dBu]');
title('Channel D - Batteries: 400kΩ vs 4MΩ');
legend([h2 h4],{'Vout2 - 400kΩ','Vout2 - 4MΩ'},'Location','best');
xlim([0 2]); ylim([-100 20]); grid on;
drawnow limitrate;


%% === Confronto dei rumori integrati ===
fprintf('=== Confronto rumore integrato ( %.2f Hz – %.2f kHz) ===\n', fmin, fmax);
gainB_alim_dB = 20*log10(mean(nB2)/mean(nB1));
gainD_alim_dB = 20*log10(mean(nD2)/mean(nD1));
gainB_batt_dB = 20*log10(mean(nB4)/mean(nB3));
gainD_batt_dB = 20*log10(mean(nD4)/mean(nD3));
fprintf('Channel B: Config2 rispetto a Config1 power supply = %.2f dB di differenza\n', gainB_alim_dB);
fprintf('Channel D: Config2 rispetto a Config1 power supply = %.2f dB di differenza\n', gainD_alim_dB);
fprintf('Channel B: Config2 rispetto a Config1 batteries = %.2f dB di differenza\n', gainB_batt_dB);
fprintf('Channel D: Config2 rispetto a Config1 batteries = %.2f dB di differenza\n', gainD_batt_dB);

%% === Optimized local function (fast + parallel) ===
function [freq, allB, allD, allA, allC, meanB, meanD, meanA, meanC, ...
          stdB, stdD, stdA, stdC, nB_uV, nD_uV] = processFolder(folder, fmin, fmax)

    files_struct = dir(fullfile(folder, '*.csv'));
    files = {files_struct.name};
    if isempty(files)
        error(['No CSV files found in ', folder]);
    end
    numFiles = numel(files);

    % === Read first file (for preallocation) ===
    firstFile = fullfile(folder, files{1});
    data = readmatrix(firstFile, 'NumHeaderLines', 2);
    freq_kHz = data(:,1);
    N = numel(freq_kHz);
    freq = freq_kHz;

    % === Preallocate ===
    allA = zeros(N, numFiles);
    allB = zeros(N, numFiles);
    allC = zeros(N, numFiles);
    allD = zeros(N, numFiles);
    nB_uV = zeros(numFiles, 1);
    nD_uV = zeros(numFiles, 1);
    df_Hz = [diff(freq_kHz); freq_kHz(end)-freq_kHz(end-1)] * 1e3;
    mask = (freq_kHz >= fmin) & (freq_kHz <= fmax);

    % === Parallel loop (uses all CPU cores) ===
    parfor k = 1:numFiles
        fname = fullfile(folder, files{k});
        M = readmatrix(fname, 'NumHeaderLines', 2);
        chA_dBu = M(:,2);
        chB_dBu = M(:,3);
        chC_dBu = M(:,4);
        chD_dBu = M(:,5);

        allA(:,k) = chA_dBu;
        allB(:,k) = chB_dBu;
        allC(:,k) = chC_dBu;
        allD(:,k) = chD_dBu;

        % Convert to Vrms once
        Vrms_B = 0.775 * 10.^(chB_dBu / 20);
        Vrms_D = 0.775 * 10.^(chD_dBu / 20);

        % Integrated noise [µV RMS]
        nB_uV(k) = sqrt(sum((Vrms_B(mask)).^2 .* df_Hz(mask))) * 1e6;
        nD_uV(k) = sqrt(sum((Vrms_D(mask)).^2 .* df_Hz(mask))) * 1e6;
    end

    % === Mean and standard deviation ===
    meanA = mean(allA, 2, 'omitnan'); stdA = std(allA, 0, 2, 'omitnan');
    meanB = mean(allB, 2, 'omitnan'); stdB = std(allB, 0, 2, 'omitnan');
    meanC = mean(allC, 2, 'omitnan'); stdC = std(allC, 0, 2, 'omitnan');
    meanD = mean(allD, 2, 'omitnan'); stdD = std(allD, 0, 2, 'omitnan');

    % === Summary ===
    disp(['--- ', folder, ' ---']);
    disp(table(files', nB_uV, nD_uV, ...
        'VariableNames', {'File','NoiseB_uVrms','NoiseD_uVrms'}));
    fprintf('Mean noise B: %.2f µV RMS (std %.2f)\n', mean(nB_uV), std(nB_uV));
    fprintf('Mean noise D: %.2f µV RMS (std %.2f)\n', mean(nD_uV), std(nD_uV));
end


