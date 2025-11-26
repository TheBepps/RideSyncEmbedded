%% ========================================================================
%  Multi-file ΔT–Voc plotting + Seebeck coefficient extraction
%
%  Loads all:
%       teg_dataOC_upX.csv
%       teg_dataOC_downX.csv
%
%  For each run:
%       - raw scatter (transparent)
%       - filtered bold curve
%       - linear regression on filtered data → Seebeck coefficient
%
%  Outputs:
%       - S values for each run
%       - mean & std for UP and DOWN
%       - overall mean & std
% ========================================================================

clear; clc; close all;

applyFilter = true;
smoothWindow = 501;

%% === Base folder ===
basePath = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\TEG\characterisation\Data\';

%% === Find all UP and DOWN files ===
upFiles   = dir(fullfile(basePath, 'teg_dataOC_up*.csv'));
downFiles = dir(fullfile(basePath, 'teg_dataOC_down*.csv'));

if isempty(upFiles) && isempty(downFiles)
    error("No teg_dataOC_upX.csv or teg_dataOC_downX.csv files found.");
end

%% === Sort numerically based on X ===
getIndex = @(fname) convertIndex(fname);

[~, idxU] = sort(arrayfun(@(f)getIndex(f.name), upFiles));
upFiles = upFiles(idxU);

[~, idxD] = sort(arrayfun(@(f)getIndex(f.name), downFiles));
downFiles = downFiles(idxD);

nUp   = numel(upFiles);
nDown = numel(downFiles);

fprintf("Found %d UP files and %d DOWN files.\n", nUp, nDown);

%% === Allocate arrays for Seebeck coefficients ===
S_up   = zeros(nUp, 1);
S_down = zeros(nDown, 1);

%% === Colormaps ===
cmapUP   = autumn(nUp);   % arancione → rosso
cmapDOWN = winter(nDown); % blu → verde

%% === Plot: create three figures (UP-only, DOWN-only, Combined fits) ===

% prealloc
S_up = zeros(nUp,1);  b_up = zeros(nUp,1);
S_down = zeros(nDown,1); b_down = zeros(nDown,1);

% keep track of DT ranges to draw fit lines
DT_min = inf; DT_max = -inf;

%% -----------------------
%  FIGURE 1: UP only
%% -----------------------
figure('Name','ΔT–Voc — UP only','Color','w'); hold on; grid on; box on;
legendUP = {};

for k = 1:nUp
    filePath = fullfile(upFiles(k).folder, upFiles(k).name);
    T = readtable(filePath);

    DT  = T.temp_delta;
    Voc = T.teg_oc_voltage;

    if applyFilter
        DT_f  = movmean(DT,  smoothWindow);
        Voc_f = movmean(Voc, smoothWindow);
    else
        DT_f = DT;   Voc_f = Voc;
    end

    % update global DT range
    DT_min = min(DT_min, min(DT_f));
    DT_max = max(DT_max, max(DT_f));

    c = cmapUP(min(k,size(cmapUP,1)), :);

    % scatter raw (transparent) - no legend entry
    scatter(DT, Voc, 10, 'MarkerEdgeColor', c, ...
                      'MarkerEdgeAlpha', 0.12, ...
                      'MarkerFaceAlpha', 0.12, ...
                      'HandleVisibility','off');

    % filtered curve (sorted)
    [DTs, idx] = sort(DT_f);
    Voc_s = Voc_f(idx);

    h = plot(DTs, Voc_s, 'Color', c, 'LineWidth', 2.2, 'LineStyle','-');
    % legend label uses actual file index extracted from filename
    fileNum = convertIndex(upFiles(k).name);
    legendUP{end+1} = sprintf('UP #%s', num2str(fileNum));

    % linear fit (store slope and intercept)
    p = polyfit(DTs, Voc_s, 1);
    S_up(k) = p(1);
    b_up(k) = p(2);
end

xlabel('Temperature Difference ΔT [°C]');
ylabel('Open-Circuit Voltage Voc [V]');
title('TEG ΔT–Voc — UP runs only');
legend(legendUP, 'Location','best');


%% -----------------------
%  FIGURE 2: DOWN only
%% -----------------------
figure('Name','ΔT–Voc — DOWN only','Color','w'); hold on; grid on; box on;
legendDOWN = {};

for k = 1:nDown
    filePath = fullfile(downFiles(k).folder, downFiles(k).name);
    T = readtable(filePath);

    DT  = T.temp_delta;
    Voc = T.teg_oc_voltage;

    if applyFilter
        DT_f  = movmean(DT,  smoothWindow);
        Voc_f = movmean(Voc, smoothWindow);
    else
        DT_f = DT;   Voc_f = Voc;
    end

    % update global DT range
    DT_min = min(DT_min, min(DT_f));
    DT_max = max(DT_max, max(DT_f));

    c = cmapDOWN(min(k,size(cmapDOWN,1)), :);

    % scatter raw (transparent)
    scatter(DT, Voc, 10, 'MarkerEdgeColor', c, ...
                      'MarkerEdgeAlpha', 0.12, ...
                      'MarkerFaceAlpha', 0.12, ...
                      'HandleVisibility','off');

    % filtered curve (sorted)
    [DTs, idx] = sort(DT_f);
    Voc_s = Voc_f(idx);

    h = plot(DTs, Voc_s, 'Color', c, 'LineWidth', 2.2, 'LineStyle','-');
    fileNum = convertIndex(downFiles(k).name);
    legendDOWN{end+1} = sprintf('DOWN #%s', num2str(fileNum));

    % linear fit (store slope and intercept)
    p = polyfit(DTs, Voc_s, 1);
    S_down(k) = p(1);
    b_down(k) = p(2);
end

xlabel('Temperature Difference ΔT [°C]');
ylabel('Open-Circuit Voltage Voc [V]');
title('TEG ΔT–Voc — DOWN runs only');
legend(legendDOWN, 'Location','best');


%% -----------------------
%  FIGURE 3: Combined — only fitted lines
%% -----------------------
figure('Name','ΔT–Voc Fits — UP & DOWN','Color','w'); hold on; grid on; box on;

% usa i nomi delle legende originali
legendFits = [legendUP(:); legendDOWN(:)];

% common DT axis for all fits
if isfinite(DT_min) && isfinite(DT_max) && (DT_max > DT_min)
    DT_common = linspace(DT_min, DT_max, 200);
else
    DT_common = linspace(0, 1, 200);
end

% ----- UP fits (solid) -----
for k = 1:nUp
    c = cmapUP(min(k,size(cmapUP,1)), :);
    Vline = S_up(k) * DT_common + b_up(k);
    plot(DT_common, Vline, 'Color', c, 'LineWidth', 2.4, 'LineStyle','-');
end

% ----- DOWN fits (solid) -----
for k = 1:nDown
    c = cmapDOWN(min(k,size(cmapDOWN,1)), :);
    Vline = S_down(k) * DT_common + b_down(k);
    plot(DT_common, Vline, 'Color', c, 'LineWidth', 2.4, 'LineStyle','-');
end

xlabel('Temperature Difference ΔT [°C]');
ylabel('Open-Circuit Voltage Voc [V]');
title('TEG ΔT–Voc Linear Fits (UP & DOWN)');
legend(legendFits, 'Location', 'best');

%% === done ===
disp("Three figures created: UP only, DOWN only, Combined fits.");

fprintf("\n================ SEEBECK COEFFICIENT RESULTS ================\n");

if nUp > 0
    fprintf("\nUP runs:\n");
    fprintf("  S_up mean = %.4f mV/°C\n", mean(S_up)*1000);
    fprintf("  S_up std  = %.4f mV/°C\n",  std(S_up)*1000);
end

if nDown > 0
    fprintf("\nDOWN runs:\n");
    fprintf("  S_down mean = %.4f mV/°C\n", mean(S_down)*1000);
    fprintf("  S_down std  = %.4f mV/°C\n",  std(S_down)*1000);
end

%% === Combined all runs ===
S_all = [S_up; S_down];

fprintf("\nALL runs combined:\n");
fprintf("  S_all mean = %.4f mV/°C\n", mean(S_all)*1000);
fprintf("  S_all std  = %.4f mV/°C\n",  std(S_all)*1000);

fprintf("\n==============================================================\n");




function idx = convertIndex(fname)
    % Estrae la parte dopo 'up' o 'down'
    token = regexp(fname, '(?<=up|down)([A-Za-z0-9]+)(?=\.csv)', 'match', 'once');

    if isempty(token)
        idx = 0;
        return;
    end

    % Se è un numero lo converte
    if all(isstrprop(token, 'digit'))
        idx = str2double(token);
        return;
    end

    % Se è una lettera → A=1, B=2, C=3 ...
    if all(isstrprop(token, 'alpha'))
        idx = upper(token) - 'A' + 1;
        return;
    end

    % fallback
    idx = 0;
end


