clc;
clear all;

addpath("model_functions/");
figures_initialization();

%% Load data
load("coefficients.mat");

%% Iterate over models and plot results

f1 = figure("Name", "Internal resistance comparison", "NumberTitle", "off");
axes1 = axes;
hold on; title(gcf().Name);
xlabel(axes1, "Mean temperature [°C]");
ylabel(axes1, "[$\Omega$]");

f2 = figure("Name", "Ideal power generation", "NumberTitle", "off");
axes2 = axes;
hold on; title(gcf().Name);
xlabel(axes2, "Delta temperature [°C]");
ylabel(axes2, "[W]");

f3 = figure("Name", "Ideal voltage generation", "NumberTitle", "off");
axes3 = axes;
hold on; title(gcf().Name);
xlabel(axes3, "Delta temperature [°C]");
ylabel(axes3, "[V]");

% Define temperature ranges
mean_T = 0:100;
delta_T = 0:100;

models = fieldnames(all_coeffs);
for kk = 1:length(models)
    
    % load current model
    model_name = models{kk};
    model_data = all_coeffs.(model_name);
    disp(model_data);

    % Evaluate models with estimated coefficients
    V_oc = open_circuit_model(model_data.seebeck, delta_T);
    R_int = internal_resistance_model(model_data.internal_resistance, mean_T);
    I_cc = V_oc ./ R_int;

    % Calculate power at specific mean temperature (Internal resistance
    % changes max power)
    R_int_at_mT_50 = internal_resistance_model(model_data.internal_resistance, 50);
    P_max = V_oc ./ R_int_at_mT_50.^2;

    % Plot one line per figure
    plot(axes1, mean_T, R_int, "DisplayName", sprintf("%s", model_name));
    plot(axes2, delta_T, P_max, "DisplayName", sprintf("%s at 50°C", model_name))
    plot(axes3, delta_T, V_oc, "DisplayName", sprintf("%s", model_name));
end

legend(axes1, "Location", "best");
legend(axes2, "Location", "best");
legend(axes3, "Location", "best");
