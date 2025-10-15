%% === Confronto di rumore e spettri di due configurazioni MAX40018 ===
% Overlay, media ± std, confronto tra configurazioni
% Conversione da dBu a nV/sqrt(Hz)
% Data: 2025-10-15

clear; clc; close all;

%% === Cartelle delle due configurazioni ===
folder1 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251014-analisi_rumore_V2-400k-Vin_Vout1_V+_Vout2';
folder2 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251014-analisi_rumore_V2-4meg-Vin_Vout1_V+_Vout2';
folder3 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251014-analisi_rumore_DC_V2-400k-Vin_Vout1_V+_Vout2';
folder4 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251014-analisi_rumore_DC_V2-4Meg-Vin_Vout1_V+_Vout2';

%% === Parametri di integrazione ===
fmin = 0.04;   % kHz -> 40 Hz
fmax = 2;      % kHz -> 2 kHz

%% === Elaborazione configurazioni ===
[f1, B1, D1, A1, C1, meanB1, meanD1, meanA1, meanC1, stdB1, stdD1, stdA1, stdC1, nB1, nD1] = processFolder(folder1, fmin, fmax);
[f2, B2, D2, A2, C2, meanB2, meanD2, meanA2, meanC2, stdB2, stdD2, stdA2, stdC2, nB2, nD2] = processFolder(folder2, fmin, fmax);
[f3, B3, D3, A3, C3, meanB3, meanD3, meanA3, meanC3, stdB3, stdD3, stdA3, stdC3, nB3, nD3] = processFolder(folder3, fmin, fmax);
[f4, B4, D4, A4, C4, meanB4, meanD4, meanA4, meanC4, stdB4, stdD4, stdA4, stdC4, nB4, nD4] = processFolder(folder4, fmin, fmax);

%% === Conversione dBu -> nV/sqrt(Hz) ===
dBu2nV = @(x) 0.775 * 10.^(x/20) * 1e9 ./ sqrt(diff([0; f1(:)]*1e3)); % nV/√Hz
% Convertiamo tutte le medie e std per overlay
convertChannel = @(meanX,stdX,fX) deal(0.775*10.^(meanX/20)*1e9 ./ sqrt([diff([0; fX(:)]*1e3)]), ...
                                       0.775*10.^(stdX/20)*1e9 ./ sqrt([diff([0; fX(:)]*1e3)]));

% Applichiamo conversione ai dati principali
[meanA1,stdA1] = convertChannel(meanA1,stdA1,f1);
[meanB1,stdB1] = convertChannel(meanB1,stdB1,f1);
[meanC1,stdC1] = convertChannel(meanC1,stdC1,f1);
[meanD1,stdD1] = convertChannel(meanD1,stdD1,f1);

[meanA2,stdA2] = convertChannel(meanA2,stdA2,f2);
[meanB2,stdB2] = convertChannel(meanB2,stdB2,f2);
[meanC2,stdC2] = convertChannel(meanC2,stdC2,f2);
[meanD2,stdD2] = convertChannel(meanD2,stdD2,f2);

[meanA3,stdA3] = convertChannel(meanA3,stdA3,f3);
[meanB3,stdB3] = convertChannel(meanB3,stdB3,f3);
[meanC3,stdC3] = convertChannel(meanC3,stdC3,f3);
[meanD3,stdD3] = convertChannel(meanD3,stdD3,f3);

[meanA4,stdA4] = convertChannel(meanA4,stdA4,f4);
[meanB4,stdB4] = convertChannel(meanB4,stdB4,f4);
[meanC4,stdC4] = convertChannel(meanC4,stdC4,f4);
[meanD4,stdD4] = convertChannel(meanD4,stdD4,f4);

%% === Overlay for each configuration (FAST MODE) ===
fastPlot = false;
maxPts = 25000;

for cfg = 1:4
    figure('Name', sprintf('Overlay Config %d', cfg), 'NumberTitle','off'); hold on;
    set(gcf, 'Renderer', 'painters');

    switch cfg
        case 1
            allA = A1; allB = B1; allC = C1; allD = D1;
            meanA_plot = meanA1; meanB_plot = meanB1; meanC_plot = meanC1; meanD_plot = meanD1;
            stdA_plot = stdA1; stdB_plot = stdB1; stdC_plot = stdC1; stdD_plot = stdD1;
            f = f1; label = '400k (Power supply)'; nB = nB1; nD = nD1;
        case 2
            allA = A2; allB = B2; allC = C2; allD = D2;
            meanA_plot = meanA2; meanB_plot = meanB2; meanC_plot = meanC2; meanD_plot = meanD2;
            stdA_plot = stdA2; stdB_plot = stdB2; stdC_plot = stdC2; stdD_plot = stdD2;
            f = f2; label = '4M (Power supply)'; nB = nB2; nD = nD2;
        case 3
            allA = A3; allB = B3; allC = C3; allD = D3;
            meanA_plot = meanA3; meanB_plot = meanB3; meanC_plot = meanC3; meanD_plot = meanD3;
            stdA_plot = stdA3; stdB_plot = stdB3; stdC_plot = stdC3; stdD_plot = stdD3;
            f = f3; label = '400k (Batteries supply)'; nB = nB3; nD = nD3;
        case 4
            allA = A4; allB = B4; allC = C4; allD = D4;
            meanA_plot = meanA4; meanB_plot = meanB4; meanC_plot = meanC4; meanD_plot = meanD4;
            stdA_plot = stdA4; stdB_plot = stdB4; stdC_plot = stdC4; stdD_plot = stdD4;
            f = f4; label = '4M (Batteries supply)'; nB = nB4; nD = nD4;
    end

    % --- Decimate ---
    if numel(f) > maxPts && fastPlot
        q = ceil(numel(f)/maxPts);
        f_plot = decimate(f,q,'fir');
        meanA_plot = decimate(meanA_plot,q,'fir'); meanB_plot = decimate(meanB_plot,q,'fir');
        meanC_plot = decimate(meanC_plot,q,'fir'); meanD_plot = decimate(meanD_plot,q,'fir');
        stdA_plot = decimate(stdA_plot,q,'fir'); stdB_plot = decimate(stdB_plot,q,'fir');
        stdC_plot = decimate(stdC_plot,q,'fir'); stdD_plot = decimate(stdD_plot,q,'fir');
    else
        f_plot = f;
    end

    % --- ±σ patch ---
    patch('XData',[f_plot; flipud(f_plot)], 'YData',[meanB_plot+stdB_plot; flipud(meanB_plot-stdB_plot)], ...
          'FaceColor','b','FaceAlpha',0.12,'EdgeColor','none');
    patch('XData',[f_plot; flipud(f_plot)], 'YData',[meanC_plot+stdC_plot; flipud(meanC_plot-stdC_plot)], ...
          'FaceColor','y','FaceAlpha',0.12,'EdgeColor','none');
    patch('XData',[f_plot; flipud(f_plot)], 'YData',[meanA_plot+stdA_plot; flipud(meanA_plot-stdA_plot)], ...
          'FaceColor',[0.6 0.6 0.6],'FaceAlpha',0.12,'EdgeColor','none');
    patch('XData',[f_plot; flipud(f_plot)], 'YData',[meanD_plot+stdD_plot; flipud(meanD_plot-stdD_plot)], ...
          'FaceColor','r','FaceAlpha',0.12,'EdgeColor','none');

    % --- Linee medie ---
    hA = plot(f_plot, meanA_plot,'k-','LineWidth',1);
    hB = plot(f_plot, meanB_plot,'b-','LineWidth',1);
    hC = plot(f_plot, meanC_plot,'y-','LineWidth',1);
    hD = plot(f_plot, meanD_plot,'r-','LineWidth',1);

    xlabel('Frequency [kHz]');
    ylabel('Voltage spectral density [nV/√Hz]');
    title({['Overlay ', label, ': Vin(A), Vout1(B), V+(C), Vout2(D)'], ...
           sprintf('Integrated Noise [nV RMS]: Vout1 = %.2f, Vout2 = %.2f', mean(nB), mean(nD))});
    legend([hA hB hC hD],{'Vin','Vout1','V+','Vout2'},'Location','best');
    grid on;
    xlim([0 2]);
    ylim([10e4 10e11]); 
    % --- Scala logaritmica ---
    set(gca, 'XScale', 'log', 'YScale', 'log');
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
xlabel('Frequency [kHz]'); ylabel('Voltage spectral density [nV/√Hz]');
title('Channel D - 4MOhm: Batteries vs Power Supply');
legend([h3 h4],{'Vout2 - Power supply','Vout2 - DC (batteries)'},'Location','best');
xlim([0 2]);
ylim([10e4 10e11]); 
grid on;
set(gca, 'XScale', 'log', 'YScale', 'log');

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
ylabel('Voltage spectral density [nV/√Hz]');
title('Channel D - Comparison: 400kOhm and 4MOhm, Batteries supply');
legend([ h2  h4],{'Vout2 - 400kOhm config','Vout2 - 4MOhm config'},'Location','best');
xlim([0 2]);
ylim([10e4 10e11]);
grid on;
set(gca, 'XScale', 'log', 'YScale', 'log');


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


