clear; clc; close all;

%% === FILE PATHS ===
fileFit = 'C:\Users\user\Documents\GitHub\RideSyncEmbedded\OpAmp\SourceMeter-sweep_DC_current-I_V_Test\fitting1.csv';
fileTest = 'C:\Users\user\Documents\GitHub\RideSyncEmbedded\OpAmp\SourceMeter-sweep_DC_current-I_V_Test\misura2.csv';

%% === READ DATA (Fit dataset) ===
opts = detectImportOptions(fileFit);
opts = setvartype(opts, {'Reading','Value'}, 'double');
Tfit = readtable(fileFit, opts);

Iin_fit  = Tfit.Value;     % [A]
Vout_fit = Tfit.Reading;   % [V]

%% === LINEAR FIT: Vout = A * Iin + B ===
p = polyfit(Iin_fit, Vout_fit, 1);
A = p(1);  % slope = V/A
B = p(2);  % intercept

Vout_fit_model = polyval(p, Iin_fit);

% R^2 calculation
SSres = sum((Vout_fit - Vout_fit_model).^2);
SStot = sum((Vout_fit - mean(Vout_fit)).^2);
R2 = 1 - SSres/SStot;

%% === PLOT FIT RESULT ===
figure('Name','Fit I-V Curve','NumberTitle','off');
hold on; grid on;
scatter(Iin_fit, Vout_fit, 25, 'b', 'filled','DisplayName','Measured data');
plot(Iin_fit, Vout_fit_model, 'r', 'LineWidth',1.5,'DisplayName','Linear fit');

xlabel('Input current [A]');
ylabel('Output voltage [V]');
title('Op-Amp I-V Characterization (DC sweep)');
legend('Location','best');

text(0.05,0.17, sprintf('V = %.4f·I + %.4f\nR^2 = %.5f', A, B, R2), ...
     'Units','normalized','VerticalAlignment','top','BackgroundColor','w');

%% === READ TEST DATA (second CSV for comparison) ===
opts2 = detectImportOptions(fileTest);
opts2 = setvartype(opts2, {'Reading','Value'}, 'double');
Ttest = readtable(fileTest, opts2);

Iin_test  = Ttest.Value;
Vout_test = Ttest.Reading;

Vout_test_model = polyval(p, Iin_test);

%% === PLOT COMPARISON ===
figure('Name','Model vs Second Measurement','NumberTitle','off');
hold on; grid on;
scatter(Iin_test, Vout_test, 20, 'k', 'filled','DisplayName','Measurement #2');
plot(Iin_test, Vout_test_model, 'r','LineWidth',1.5,'DisplayName','Model prediction');

xlabel('Input current [A]');
ylabel('Output voltage [V]');
title('Comparison: Fit vs Second Dataset');
legend('Location','best');

%% === RESIDUALS PLOT ===
residuals = Vout_test - Vout_test_model;

figure('Name','Residuals','NumberTitle','off');
stem(Iin_test, residuals, 'filled');
grid on;
xlabel('Input current [A]');
ylabel('Residual [V]');
title('Residuals: Measured - Model');

text(0.05,0.12, sprintf('Residuals mean = %.6f\n', mean(residuals)), ...
     'Units','normalized','VerticalAlignment','top','BackgroundColor','w');

figure('Name','Residuals Normality Check','NumberTitle','off');

% 1) Histogram + Gaussian fit
subplot(1,2,1);
histogram(residuals, 'Normalization', 'pdf', 'FaceColor', [0.5 0.7 1], 'EdgeColor','k');
hold on;
mu = mean(residuals);
sigma = std(residuals);
xg = linspace(min(residuals), max(residuals), 100);
yg = normpdf(xg, mu, sigma);
plot(xg, yg, 'r', 'LineWidth', 1.5);
title('Residuals Histogram + Gaussian PDF');
xlabel('Residual value');
ylabel('Probability density');
legend('Residuals','Gaussian fit','Location','best');
grid on;

% 2) QQ-plot
subplot(1,2,2);
qqplot(residuals);
title('QQ-Plot of Residuals');
grid on;


%% === PRINT RESULTS ===
fprintf('\n=== LINEAR MODEL RESULT (FREE FIT) ===\n');
fprintf('Model:  Vout = A * Iin + B\n');
fprintf('A = %.6f  [V/A]\n', A);
fprintf('B = %.6f  [V]\n', B);
fprintf('R^2 = %.6f\n', R2);
fprintf('-------------------------------------\n');
fprintf('Residuals mean = %.6f\n', mean(residuals));
fprintf('=====================================\n\n');
