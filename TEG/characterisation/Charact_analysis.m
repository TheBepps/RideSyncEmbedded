%% =========================================================================
%  Seebeck Model Training + Validation
%
%  - Loads all teg_dataOC_up*.csv and teg_dataOC_down*.csv
%  - Uses 3 UP + 3 DOWN runs for model training
%  - Fits linear Seebeck model: Voc = S * ΔT + b
%  - Uses remaining runs for validation
%  - Computes RMSE, MAE, R²
%  - Generates training and validation plots
% =========================================================================

clear; clc; close all;

%% === BASE FOLDER ===
basePath = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\TEG\characterisation\Data\';

%% === FIND FILES ===
upFiles   = dir(fullfile(basePath, 'teg_dataOC_up*.csv'));
downFiles = dir(fullfile(basePath, 'teg_dataOC_down*.csv'));

if isempty(upFiles) || isempty(downFiles)
    error("Not enough UP or DOWN files found.");
end

%% === SORT NUMERICALLY BASED ON INDEX ===
getIndex = @(fname) convertIndex(fname);

[~, idxU] = sort(arrayfun(@(f)getIndex(f.name), upFiles));
[~, idxD] = sort(arrayfun(@(f)getIndex(f.name), downFiles));

upFiles   = upFiles(idxU);
downFiles = downFiles(idxD);

nUp   = numel(upFiles);
nDown = numel(downFiles);

fprintf("Found %d UP files and %d DOWN files.\n", nUp, nDown);

%% ========================================================================
%  SPLIT INTO TRAINING (3+3) AND VALIDATION (remaining)
% ========================================================================
nTrain = 3;

trainUP   = upFiles(1:nTrain);
trainDOWN = downFiles(1:nTrain);

validUP   = upFiles(nTrain+1:end);
validDOWN = downFiles(nTrain+1:end);

fprintf("Training set:   %d UP, %d DOWN\n", numel(trainUP), numel(trainDOWN));
fprintf("Validation set: %d UP, %d DOWN\n", numel(validUP), numel(validDOWN));

%% === FILTER SETTINGS ===
applyFilter  = false;
smoothWindow = 501;

%% ========================================================================
%  LOAD TRAINING DATA + FIT MODEL
% ========================================================================
DT_train = [];
Voc_train = [];

% --- collect training data (UP) ---
for k = 1:numel(trainUP)
    [DTs, Voc_s] = loadAndSmooth(fullfile(trainUP(k).folder, trainUP(k).name), applyFilter, smoothWindow);
    DT_train  = [DT_train;  DTs];
    Voc_train = [Voc_train; Voc_s];
end

% --- collect training data (DOWN) ---
for k = 1:numel(trainDOWN)
    [DTs, Voc_s] = loadAndSmooth(fullfile(trainDOWN(k).folder, trainDOWN(k).name), applyFilter, smoothWindow);
    DT_train  = [DT_train;  DTs];
    Voc_train = [Voc_train; Voc_s];
end

% === Fit linear Seebeck model ===
model = polyfit(DT_train, Voc_train, 1);
S_model = model(1);
b_model = model(2);

fprintf("\n=== TRAINED MODEL ===\n");
fprintf("Seebeck S = %.6f V/°C\n", S_model);
fprintf("Intercept b = %.6f V\n", b_model);

%% ========================================================================
%  PLOT TRAINING DATA + MODEL
% ========================================================================
figure('Name','Training Data + Seebeck Model','Color','w'); hold on; grid on; box on;

scatter(DT_train, Voc_train, 8, 'k', 'filled', 'MarkerFaceAlpha', 0.1);

DT_line = linspace(min(DT_train), max(DT_train), 200);
Voc_line = polyval(model, DT_line);

plot(DT_line, Voc_line, 'r', 'LineWidth', 2.5);

xlabel('Temperature Difference ΔT [°C]');
ylabel('Open-Circuit Voltage Voc [V]');
title( sprintf('Training Data (%d UP + %d DOWN) and Fitted Seebeck Line', nTrain, nTrain) );
legend('Training Data','Fit: Voc = S·ΔT + b','Location','best');

%% ========================================================================
%  VALIDATION
% ========================================================================
DT_valid = [];
Voc_valid = [];

validationFiles = [validUP; validDOWN];

for k = 1:numel(validationFiles)
    [DTs, Voc_s] = loadAndSmooth(fullfile(validationFiles(k).folder, validationFiles(k).name), applyFilter, smoothWindow);
    DT_valid  = [DT_valid;  DTs];
    Voc_valid = [Voc_valid; Voc_s];
end

% model predictions
Voc_pred = polyval(model, DT_valid);

%residuals
residuals = Voc_valid - Voc_pred;

% === Validation Metrics ===
RMSE = sqrt(mean((Voc_pred - Voc_valid).^2));
MAE  = mean(abs(Voc_pred - Voc_valid));
R2   = 1 - sum((Voc_valid - Voc_pred).^2) / sum((Voc_valid - mean(Voc_valid)).^2);

fprintf("\n=== VALIDATION RESULTS ===\n");
fprintf("RMSE = %.6f V\n", RMSE);
fprintf("MAE  = %.6f V\n", MAE);
fprintf("R²   = %.4f\n", R2);

%% ========================================================================
%  VALIDATION PLOT
% ========================================================================
figure('Name','Model Validation','Color','w'); hold on; grid on; box on;

scatter(DT_valid, Voc_valid, 20, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
plot(DT_line, Voc_line, 'r', 'LineWidth', 2.5);

xlabel('Temperature Difference ΔT [°C]');
ylabel('Open-Circuit Voltage Voc [V]');
title('Validation Data vs. Trained Seebeck Model');
legend('Validation Data','Trained Model','Location','best');

fprintf("\n=== DONE ===\n");

%% ===============================================================
%  RESIDUAL DIAGNOSTICS (Q–Q Plot, Histogram, Residual Trends)
% ===============================================================

%figure('Name','Residual Diagnostics','Color','w');

%% === 1) Q–Q Plot of Residuals ===
figure('Name','Q–Q Plot of Residuals','Color','w'); hold on; grid on; box on;
%subplot(1,3,1);
qqplot(residuals);
title('Q–Q Plot of Residuals');
grid on;

%% === 2) Residual Histogram + Gaussian Fit ===
figure('Name','Residual Histogram + Gaussian Fit','Color','w'); hold on; grid on; box on;
%subplot(1,3,2);
hold on; grid on; box on;

% Histogram (normalized)
histogram(residuals, 'Normalization', 'pdf', ...
    'FaceColor', [0.2 0.4 0.9], 'FaceAlpha', 0.4);

% Fit Gaussian PDF
mu  = mean(residuals);
sig = std(residuals);
xPDF = linspace(mu - 4*sig, mu + 4*sig, 200);
pdf_gauss = normpdf(xPDF, mu, sig);

plot(xPDF, pdf_gauss, 'r', 'LineWidth', 2);

title('Residual Histogram + Gaussian PDF');
xlabel('Residual');
ylabel('Probability Density');
legend('Residuals Histogram','Gaussian Fit');

%% === 3) Residuals vs ΔT (checks heteroscedasticity) ===
figure('Name','Residuals vs ΔT','Color','w'); hold on; grid on; box on;
%subplot(1,3,3);
scatter(DT_valid, residuals, 15, 'filled', ...
    'MarkerFaceAlpha', 0.5, 'MarkerFaceColor', [0 0.6 0.2]);
hold on;
yline(0, 'k--', 'LineWidth', 1.2);
grid on; box on;
xlabel('\DeltaT [°C]');
ylabel('Residual [V]');
title('Residuals vs \DeltaT');

%% === Print summary to terminal ===
fprintf("\n=== Residual Diagnostics ===\n");
fprintf("Residual mean      = %.6f V\n", mean(residuals));
fprintf("Residual std       = %.6f V\n", std(residuals));
fprintf("Residual max abs   = %.6f V\n", max(abs(residuals)));
fprintf("Normality check: Q–Q plot required.\n");
fprintf("=================================================\n");




%% ========================================================================
%  FUNCTION TO LOAD AND PROCESS A FILE
% ========================================================================
function [DTs, Voc_s] = loadAndSmooth(filePath, applyFilter, smoothWindow)

    T = readtable(filePath);
    DT  = T.temp_delta;
    Voc = T.teg_oc_voltage;

    if applyFilter
        DT_f  = movmean(DT,  smoothWindow);
        Voc_f = movmean(Voc, smoothWindow);
    else
        DT_f = DT;
        Voc_f = Voc;
    end

    [DTs, idx] = sort(DT_f);
    Voc_s = Voc_f(idx);
end

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


