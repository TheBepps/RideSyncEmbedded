clc;
clear all;

addpath("model_functions/");
%% Figures style initialization
figures_initialization();
%% --- Config ---- %%

% Data location
base_folder = "../data/"; 
% These are the names of the subfolders, that must correspond to the TEG model.
models = ["teg_mat", "teg_12706", "teg_vl25"];
selected_model = 1;
% Names of the datasets of this model
oc_file_name = "open_circuit.csv";
cc_file_name =  "1k_ohm_load.csv";

% Flag to enable automatic figure export.
% This is used when writing a paper/presentation. 
% Exported figures are in saved_figures/ folder.
enableSaveFig = 0;

load_value = 995; % 995;
%load_value = 9955; % 10kOhm

%% Data loading
model_name = models(selected_model);

% Load CSV datasets of "selected_model"
model_folder = strcat(base_folder, strcat(model_name, "/"));
oc_data = readtable(strcat(model_folder, oc_file_name));
cc_data = readtable(strcat(model_folder, cc_file_name));


% Define sample structure for estimated coefficients
coeffs_struct.seebeck = [0];
coeffs_struct.internal_resistance = [0 0];
all_coeffs.(model_name) = coeffs_struct;

% Load or save coefficients.mat (Used in model_comparison.m)
coeffs_table_path = fullfile(pwd, "coefficients.mat");
if(~isfile(coeffs_table_path))
  save(coeffs_table_path, "all_coeffs");
else
  load(coeffs_table_path);
end

%% Plot Dataset
f = newFig('Dataset plots');
t = tiledlayout(2, 2, 'TileSpacing','tight');
% voltage
nexttile; hold on;
oc_line = plot(oc_data.temp_hot_side, oc_data.temp_cold_side, 'DisplayName', 'open circuit dataset');
cc_line = plot(cc_data.temp_hot_side, cc_data.temp_cold_side, 'DisplayName', 'internal resistance dataset');
oc_color = get(oc_line, 'Color');
cc_color = get(cc_line, 'Color');
ylabel('cold side $[^\circ C]$');
xlabel('hot side $[^\circ C]$');
axis equal
legend('Location', 'northwest');

nexttile; hold on;
oc_dT = oc_data.temp_hot_side - oc_data.temp_cold_side;
oc_mT = (oc_data.temp_hot_side + oc_data.temp_cold_side) / 2;
cc_dT = cc_data.temp_hot_side - cc_data.temp_cold_side;
cc_mT = (cc_data.temp_hot_side + cc_data.temp_cold_side) / 2;
plot(oc_mT, oc_dT,  'DisplayName', 'open circuit dataset');
plot(cc_mT, cc_dT, 'DisplayName', 'internal resistance dataset');
ylabel('delta temperature $[^\circ C]$');
xlabel('mean temperature $[^\circ C]$');
axis equal
legend('Location', 'south');

nexttile;
plot(oc_dT, oc_data.teg_oc_voltage, 'Color',oc_color, 'DisplayName', 'open circuit dataset');
xlabel('delta temperature $[^\circ C]$');
ylabel('voltage [V]');
legend('Location', 'northwest');
title("Voltage");

nexttile;
plot(cc_dT, cc_data.teg_current * 1e3, 'Color',cc_color, 'DisplayName', 'internal resistance dataset');
xlabel('delta temperature $[^\circ C]$');
ylabel('current [mA]');
legend('Location', 'south');
title("Current");
title(t, gcf().Name);
saveFigForReport(enableSaveFig);

newFig('Open Circuit Voltage');
plot(oc_data.temp_hot_side - oc_data.temp_cold_side, oc_data.teg_oc_voltage);
xlabel("delta temperature");
ylabel("open circuit voltage");
saveFigForReport(enableSaveFig);

%% If needed remove outliers
filter_data = 0;
if filter_data == 1
    dT = oc_data.temp_hot_side - oc_data.temp_cold_side;
    [b,a] = butter(8, 0.01);
    fdT = filtfilt(b, a, dT);
    variation = [0.0; diff(fdT)];
    
    thr2 = [6105, 6305];
    tmp = oc_data.time / 1e3;
    t_mask = (tmp > thr2(1)) & (tmp < thr2(2));
    
    newFig('Outliers cleanup');
    clf;
    t = tiledlayout(1, 2);
    nexttile;
    hold on;
    plot(tmp, variation, '-');
    xline(thr2, 'r');
    legend({'Temperature variation','Time threshold'});
    xlabel("time [s]");
    ylabel("$^\circ C / s $");
    grid on;
    nexttile;
    hold on;
    plot(dT, oc_data.teg_oc_voltage, '.', 'DisplayName','measurements');
    % plot(dT(mask), V(mask), 'o', 'DisplayName','removed by variation');
    plot(dT(t_mask), oc_data.teg_oc_voltage(t_mask), '.', 'DisplayName','samples removed');
    legend;
    xlabel("mean temperature [$^\circ C$]");
    ylabel("open circuit voltage [V]");
    grid on;
    title(t, gcf().Name);
    saveFigForReport(enableSaveFig);

    oc_data(t_mask,:) = [];
end

%% Seeback model

dT = oc_data.temp_hot_side - oc_data.temp_cold_side;
mT = (oc_data.temp_hot_side + oc_data.temp_cold_side) ./ 2;
V = oc_data.teg_oc_voltage;

tbl = table(dT, V);

mdl = @(coeffs, delta_T)(open_circuit_model(coeffs, delta_T));
nlm = fitnlm(tbl, mdl, 0.0,'Options',statset('Display','final','Robust','On'));
fitted_seebeck_coeffs = nlm.Coefficients.Estimate;

all_coeffs.(model_name).seebeck = fitted_seebeck_coeffs;
save(coeffs_table_path, "all_coeffs");

disp("------------------");
disp(fitted_seebeck_coeffs);
disp("------------------");
%%
newFig('seebeck effect');
plot(dT, V, '.', 'DisplayName','measurements');
plot(dT, open_circuit_model(fitted_seebeck_coeffs, dT), '-', 'DisplayName','fitted', 'LineWidth',3);
legend;
xlabel("mean temperature");
ylabel("open circuit voltage");
title(gcf().Name);
saveFigForReport(enableSaveFig);
grid on;

%% Internal resistance 
th = cc_data.temp_hot_side;
tc = cc_data.temp_cold_side;
I = cc_data.teg_current;
dT = th - tc;
mT = (th + tc) / 2;
Voc = open_circuit_model(fitted_seebeck_coeffs, dT);

% Remove samples with saturated current.
% Current was at maximum scale of the ADC
mask = I > 1.7e-3;
I(mask) = [];
th(mask) = [];
tc(mask) = [];
dT(mask) = [];
mT(mask) = [];
Voc(mask) = [];

load_value_arr = ones(size(mT))*load_value;

residuals = @(coeffs)(residuals_function(I, current_model(coeffs, [mT, load_value_arr, Voc])));
default_options = optimoptions('fmincon');
fitted_internal_resistance_coeffs = fmincon(residuals, ...
  [0.5, 0.1], ...
  [],[],[],[], ...
  [0, 0], ...
  [10, 5], ...
  [], ...
  default_options ...
);

all_coeffs.(model_name).internal_resistance = fitted_internal_resistance_coeffs;
save(coeffs_table_path, "all_coeffs");

fig = newFig("Open Circuit Voltage VS Current");
yyaxis left
plot(dT, Voc, "LineWidth", 3, "DisplayName", "Open Circuit Voltage");
ylabel("[V]");
yyaxis right
plot(dT, I, '.', "DisplayName", "Current");
legend('Location','best');
xlabel("Mean Temperature $[^\circ C]$");
ylabel("[A]");
title(gcf().Name);
saveFigForReport(enableSaveFig);

newFig("Current model");
hold on;
plot(dT, I * 1000, '.', "DisplayName", "Measured current");
plot(dT, current_model(fitted_internal_resistance_coeffs, [mT, load_value_arr, Voc]) * 1000, "DisplayName", "Estimated from model", "LineWidth",3);
xlabel("$\Delta T [^\circ C]$");
ylabel("[mA]");
grid on;
title(gcf().Name);
legend();
hold off;
saveFigForReport(enableSaveFig);

newFig("Estimated internal resistance");
plot(mT, internal_resistance_model(fitted_internal_resistance_coeffs, mT), "DisplayName", "Internal resistance");
xlabel("Mean temperature $[^\circ C]$");
ylabel("Internal resistance $[\Omega]$");
grid on;
title(gcf().Name);
legend();
saveFigForReport(enableSaveFig);


%% Print results

disp("---- Results ----");
fprintf("Model: %s\n", model_name);
fprintf("Data files loaded in folder: %s\n", model_folder);
fprintf("Data loaded: %s, %s\n", oc_file_name, cc_file_name);
fprintf("\n");
fprintf("Seebeck coefficient %f\n", fitted_seebeck_coeffs(1));
fprintf("Internal resistance: %f\t%f\n", fitted_internal_resistance_coeffs(1), fitted_internal_resistance_coeffs(2));
disp("-----------------");


%% Helper functions (must be at the bottom)
function fig = newFig(name)
    fig = figure("Name", name, 'NumberTitle','off');
    clf; hold on;
end
function [] = saveFigForReport(enabled)
  if enabled == 1
      name = gcf().Name;
      name = strrep(name, ":", "");
      basepath = 'saved_figures/';
      extension = '.epsc';
      fullpath = strcat(basepath, name);
      print(fullpath, "-r400", "-dpng");
  end
end
