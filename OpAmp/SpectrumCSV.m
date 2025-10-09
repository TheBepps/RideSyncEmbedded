%% === Analisi spettro OpAmp MAX40018 ===
% Lettura e plot dei due canali da file CSV
% Autore: [tuo nome]
% Data: 2025-10-09

clear; clc; close all;

% === Path del file CSV ===
fname = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\OpAmp\spectrum_max40018-differential\20251008-analisi_rumore-400k.csv';

% === Lettura file con due righe di intestazione ===
opts = detectImportOptions(fname, 'NumHeaderLines', 2);
opts.VariableNamesLine = 2;
T = readtable(fname, opts);

% Controllo colonne
if all(ismember({'Frequency','ChannelB','ChannelD'}, T.Properties.VariableNames))
    freq_kHz = T.Frequency;
    chB_dBu = T.ChannelB;
    chD_dBu = T.ChannelD;
else
    % In caso di nomi leggermente diversi (es. spazi o parentesi)
    freq_kHz = T{:,1};
    chB_dBu = T{:,2};
    chD_dBu = T{:,3};
end

%% === Plot Canale B (Vout1) ===
figure(1);
plot(freq_kHz, chB_dBu, 'b', 'LineWidth', 1.3);
grid on;
xlabel('Frequenza [kHz]');
ylabel('Ampiezza [dBu]');
title('Spettro Vout1 (Channel B)');
set(gca, 'XScale', 'log'); % se vuoi scala logaritmica
legend('Channel B');

%% === Plot Canale D (Vout2) ===
figure(2);
plot(freq_kHz, chD_dBu, 'r', 'LineWidth', 1.3);
grid on;
xlabel('Frequenza [kHz]');
ylabel('Ampiezza [dBu]');
title('Spettro Vout2 (Channel D)');
set(gca, 'XScale', 'log'); % scala logaritmica
legend('Channel D');

