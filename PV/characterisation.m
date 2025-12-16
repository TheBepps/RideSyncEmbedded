%% ============================================================
%  PV characterization – P–V curves with error bars
% ============================================================

clear; clc; close all;

basePath = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\PV\characterisation\';

files = {
    '1v.csv',   1.0
    '2v.csv',   2.0
    '3v.csv',   3.0
    '4v.csv',   4.0
    '4_7v.csv', 4.7
};

nTests = size(files,1);
colors = lines(nTests);
legendEntries = cell(nTests,1);

samples_per_step = 200;   % <<< come nel TEG

%% === OUTPUT FIGURE ===
figure('Name','PV P–V Characteristics (with error bars)','Color','w');
hold on; grid on; box on;
xlabel('Voltage [V]');
ylabel('Power [W]');
title('PV P–V Curves');

%% === SUMMARY TABLE ===
summary = table('Size',[nTests 4], ...
    'VariableTypes',{'double','double','double','double'}, ...
    'VariableNames',{'Voc','Vmpp','Pmpp','PmppStd'});

%% === MAIN LOOP ===
for n = 1:nTests

    file = fullfile(basePath, files{n,1});
    Voc_nom = files{n,2};

    %% === Load data ===
    T = readtable(file);

    V = T.Value;
    P = abs(T.Reading);

    %% === Group samples per step ===
    n_steps = floor(length(V)/samples_per_step);

    Vmean = zeros(n_steps,1);
    Pmean = zeros(n_steps,1);
    Pstd  = zeros(n_steps,1);

    for k = 1:n_steps
        idx = (k-1)*samples_per_step + (1:samples_per_step);

        Vk = V(idx);
        Pk = P(idx);

        Vmean(k) = mean(Vk);
        Pmean(k) = mean(Pk);
        Pstd(k)  = std(Pk);
    end
    PVdata(n).V = Vmean;
    PVdata(n).P = Pmean;
    PVdata(n).Pstd = Pstd;
    PVdata(n).Voc = Voc_nom;


    %% === Sort by voltage ===
    [Vmean, idx] = sort(Vmean);
    Pmean = Pmean(idx);
    Pstd  = Pstd(idx);

    %% === Identify MPP ===
    [Pmpp, idxMPP] = max(Pmean);
    Vmpp = Vmean(idxMPP);
    PmppStd = Pstd(idxMPP);

    %% === Plot curve with error bars ===
    errorbar(Vmean, Pmean, Pstd, ...
        '-', ...
        'Color', colors(n,:), ...
        'LineWidth', 1.2, ...
        'MarkerSize', 4, ...
        'CapSize', 5 ...
    );

    % Mark MPP
    plot(Vmpp, Pmpp, 'ko', ...
        'MarkerFaceColor','r', ...
        'MarkerSize',6, ...
        'HandleVisibility','off');

    %% === Legend entry (PV-style, not TEG) ===
    legendEntries{n} = sprintf( ...
        'Voc ≈ %.1f V | Vmpp = %.2f V | Pmpp = %.3f mW', ...
        Voc_nom, Vmpp, 1e3*Pmpp);

    %% === Store summary ===
    summary(n,:) = {Voc_nom, Vmpp, Pmpp, PmppStd};

end

%% === Final legend ===

axMain = gca;
axis(axMain,'tight');

pbaspect(axMain,[16 9 1]);   % oppure [4 3 1] se vuoi più largo

legend(legendEntries, 'Location','northwest');

%% ============================================================
%  INSET: Zoom on low-Voc cases (1 V and 2 V)
% ============================================================

% Create inset axes (normalized figure coordinates)
axInset = axes('Position',[0.19 0.3 0.25 0.28]);  % [x y w h]
hold(axInset,'on'); grid(axInset,'on'); box(axInset,'on');

xlim(axInset, [0 2.1]);
ylim(axInset, [0 1.2*max(summary.Pmpp(summary.Voc <= 2))]);

axis(axInset,'tight');
pbaspect(axInset, pbaspect(axMain));

xlabel(axInset,'Voltage [V]');
ylabel(axInset,'Power [W]');
title(axInset,'Zoom: Voc = 1–2 V');

% Plot only Voc = 1 V and 2 V
for n = 1:nTests
    if PVdata(n).Voc <= 2.0

        errorbar(axInset, ...
            PVdata(n).V, ...
            PVdata(n).P, ...
            PVdata(n).Pstd, ...
            '-', ...
            'Color', colors(n,:), ...
            'LineWidth', 1.2, ...
            'CapSize', 4 ...
        );

        % Mark MPP
        [Pmpp, idx] = max(PVdata(n).P);
        Vmpp = PVdata(n).V(idx);
        plot(axInset, Vmpp, Pmpp, 'ro', ...
            'MarkerFaceColor','r', 'MarkerSize',5);
    end
end


%%
figure('Name','MPP Ratio Stability','Color','w');
grid on; box on;
plot(summary.Voc, summary.Vmpp ./ summary.Voc, 'o-', 'LineWidth', 2);
xlabel('Voc [V]');
ylabel('Vmpp / Voc');
title('MPP Voltage Ratio (Experimental)');
ylim([0.6 0.9]);


disp('================ PV SUMMARY ================');
disp(summary);
