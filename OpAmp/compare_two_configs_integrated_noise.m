%% === Confronto di rumore e spettri di due configurazioni MAX40018 ===
% Overlay, media ± std, confronto tra configurazioni
% Data: 2025-10-09

clear; clc; close all;

%% === Cartelle delle due configurazioni ===
folder1 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251008-analisi_rumore-400k';
folder2 = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251009-20251008-analisi_rumore-4meg-Vin_Vout1_V+_Vout2';

%% === Parametri di integrazione ===
fmin = 0.04;   % kHz -> 50 Hz
fmax = 10;     % kHz -> 10 kHz

%% === Elaborazione configurazioni ===
[f1, B1, D1, A, C, meanB1, meanD1, meanA, meanC, stdB1, stdD1, stdA, stdC, nB1, nD1] = processFolder(folder1, fmin, fmax);
[f2, B2, D2, ~, ~, meanB2, meanD2, ~, ~, stdB2, stdD2, ~, ~, nB2, nD2] = processFolder(folder2, fmin, fmax);

%% === Overlay per ciascuna configurazione ===
for cfg = 1:2
    figure('Name',sprintf('Overlay Config %d',cfg),'NumberTitle','off'); hold on;
    if cfg==1
        allA = A; allB = B1; allC = C; allD = D1; 
        meanA_plt = meanA; meanB_plt = meanB1; meanC_plt = meanC; meanD_plt = meanD1;
        stdA_plt  = stdA;  stdB_plt  = stdB1;  stdC_plt  = stdC;  stdD_plt  = stdD1;
        f = f1; label='400k';
    else
        allA = A; allB = B2; allC = C; allD = D2;
        meanA_plt = meanA; meanB_plt = meanB2; meanC_plt = meanC; meanD_plt = meanD2;
        stdA_plt  = stdA;  stdB_plt  = stdB2;  stdC_plt  = stdC;  stdD_plt  = stdD2;
        f = f2; label='4M';
    end

    % Spettri trasparenti
    plot(f, allA, 'Color', [0.6 0.6 0.6]);
    plot(f, allB, 'Color', [0.7 0.85 1]);
    plot(f, allC, 'Color', [1 1 0.6]);
    plot(f, allD, 'Color', [1 0.8 0.8]);

    % Patch ±σ
    fillX = [f; flipud(f)];
    fillY_A = [meanA_plt+stdA_plt; flipud(meanA_plt-stdA_plt)];
    fillY_B = [meanB_plt+stdB_plt; flipud(meanB_plt-stdB_plt)];
    fillY_C = [meanC_plt+stdC_plt; flipud(meanC_plt-stdC_plt)];
    fillY_D = [meanD_plt+stdD_plt; flipud(meanD_plt-stdD_plt)];

    patch('XData', fillX, 'YData', fillY_A, 'FaceColor',[0.6 0.6 0.6],'FaceAlpha',0.15,'EdgeColor','none');
    patch('XData', fillX, 'YData', fillY_B, 'FaceColor','b','FaceAlpha',0.15,'EdgeColor','none');
    patch('XData', fillX, 'YData', fillY_C, 'FaceColor','y','FaceAlpha',0.15,'EdgeColor','none');
    patch('XData', fillX, 'YData', fillY_D, 'FaceColor','r','FaceAlpha',0.12,'EdgeColor','none');

    % Linee medie
    plot(f, meanA_plt, 'k-', 'LineWidth',1.5); % Vin
    plot(f, meanB_plt, 'b-', 'LineWidth',1.5); % Vout1
    plot(f, meanC_plt, 'y-', 'LineWidth',1.5); % V+
    plot(f, meanD_plt, 'r-', 'LineWidth',1.5); % Vout2

    xlabel('Frequenza [kHz]'); ylabel('Ampiezza [dBu]');
    title(['Overlay Config ',label,': Vin(A), Vout1(B), V+(C), Vout2(D)']);
    legend({'Vin','Vout1','V+','Vout2'},'Location','best');
end

%% === Confronto tra configurazioni per ciascun canale ===
for ch = ["B","D"]
    figure('Name',['Confronto Channel ',ch],'NumberTitle','off'); hold on;
    if ch=="B"
        fillY1 = [meanB1+stdB1; flipud(meanB1-stdB1)];
        fillY2 = [meanB2+stdB2; flipud(meanB2-stdB2)];
        mean1 = meanB1; mean2 = meanB2;
        col1='b'; col2='g';
        title_txt = 'Channel B';
    else
        fillY1 = [meanD1+stdD1; flipud(meanD1-stdD1)];
        fillY2 = [meanD2+stdD2; flipud(meanD2-stdD2)];
        mean1 = meanD1; mean2 = meanD2;
        col1='r'; col2='m';
        title_txt = 'Channel D';
    end
    fillX = [f1; flipud(f1)];
    patch('XData', fillX, 'YData', fillY1, 'FaceColor',col1,'FaceAlpha',0.15,'EdgeColor','none');
    patch('XData', fillX, 'YData', fillY2, 'FaceColor',col2,'FaceAlpha',0.15,'EdgeColor','none');
    h1=plot(f1, mean1, [col1,'-'],'LineWidth',1.5);
    h2=plot(f1, mean2, [col2,'-'],'LineWidth',1.5);
    xlabel('Frequenza [kHz]'); ylabel('Ampiezza [dBu]');
    title(['Confronto ',title_txt,' tra Config1 e Config2']);
    legend([h1 h2], {'Config 400k','Config 4M'},'Location','best');
end

%% === Confronto dei rumori integrati ===
disp('=== Confronto rumore integrato (50 Hz – 10 kHz) ===');
gainB_dB = 20*log10(mean(nB2)/mean(nB1));
gainD_dB = 20*log10(mean(nD2)/mean(nD1));
fprintf('Channel B: Config2 rispetto a Config1 = %.2f dB di differenza\n', gainB_dB);
fprintf('Channel D: Config2 rispetto a Config1 = %.2f dB di differenza\n', gainD_dB);

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
