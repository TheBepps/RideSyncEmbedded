clear; clc; close all;

%% ================= PATHS =================
PVpath  = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\Final test\ft1\Power PV\ft1 v2';
TEGpath = 'C:\Users\Admin\Documents\GitHub\RideSyncEmbedded\Final test\ft1\Power TEG';

%% ================= FUNCTION =================
analyzeFolder = @(basePath) ...
    analyzeKeithleyFolder(basePath);

%% ================= RUN =================
PV = analyzeFolder(PVpath);
TEGall = analyzeFolder(TEGpath);

%% ================= SPLIT TEG =================
fname = erase(TEGall.File,'.csv');

isBQ  = endsWith(fname, ["21","22","23","z2"]);
isLTC = endsWith(fname, ["1","2","3","z"]) & ~isBQ;


TEG_LTC = TEGall(isLTC,:);
TEG_BQ  = TEGall(isBQ,:);

%% ================= DISPLAY =================
disp('========= PV RESULTS ========='); disp(PV);
disp('========= TEG – LTC ========='); disp(TEG_LTC);
disp('========= TEG – BQ =========');  disp(TEG_BQ);

%% ================= PLOT =================
% ---------------- PV ----------------
Vbatt = [3.4, 4.8, 2.9];  % PV voltage for 1A

PV_1A = PV(contains(PV.File,'1a'),:);
PV_1B = PV(contains(PV.File,'1b'),:);

figure('Name','Power PV Group 1A','Color','w'); hold on; grid on; box on;
plot(Vbatt, PV_1A.MeanPower_W,'o','MarkerSize',8,'MarkerFaceColor','r');
xlabel('Vbatt [V]'); ylabel('Mean Power [W]');
title('Power PV: Voc = 4 V - 100 W/m^2');
legend('BQ harvester','Location','southeast');

Vbatt = [3.4, 4.8, 3.7];  % PV voltage for 1B

figure('Name','Power PV Group 1B','Color','w'); hold on; grid on; box on;
plot(Vbatt, PV_1B.MeanPower_W,'o','MarkerSize',8,'MarkerFaceColor','r');
xlabel('Vbatt [V]'); ylabel('Mean Power [W]');
title('Power PV: Voc = 4.7 V - 1000 W/m^2');
legend('BQ harvester','Location','southeast');

% ---------------- TEG-LTC ----------------
TEG_LTC_X = TEG_LTC(contains(TEG_LTC.File,'x'),:);
TEG_LTC_Y = TEG_LTC(contains(TEG_LTC.File,'y'),:);
TEG_BQ_X = TEG_BQ(contains(TEG_BQ.File,'x'),:);
TEG_BQ_Y = TEG_BQ(contains(TEG_BQ.File,'y'),:);

Vstore = [3.4, 4.8, 2.9];  % TEG voltage for 1X and 1Y

figure('Name','Power TEG LTC X','Color','w'); hold on; grid on; box on;
plot(Vstore, TEG_LTC_X.MeanPower_W,'o','MarkerSize',8,'MarkerFaceColor','r');
plot(Vstore, TEG_BQ_X.MeanPower_W,'o','MarkerSize',8,'MarkerFaceColor','b');
title('Power TEG LTC @ ΔT = 20 °C');
xlabel('Vstore [V]'); ylabel('Mean Power [W]');
legend('LTC harvester', 'BQ harvester','Location','best');

figure('Name','Power TEG LTC Y','Color','w'); hold on; grid on; box on;
plot(Vstore, TEG_LTC_Y.MeanPower_W,'o','MarkerSize',8,'MarkerFaceColor','r');
plot(Vstore, TEG_BQ_Y.MeanPower_W,'o','MarkerSize',8,'MarkerFaceColor','b');
title('Power TEG LTC @ ΔT = 50 °C');
xlabel('Vstore [V]'); ylabel('Mean Power [W]');
legend('LTC harvester', 'BQ harvester','Location','best');

% ---------------- TEG LTC vs BQ totale ----------------
figure('Name','Power TEG LTC vs BQ','Color','w'); hold on; grid on; box on;
plot(TEG_LTC.MeanPower_W,'o-','LineWidth',1.8);
plot(TEG_BQ.MeanPower_W,'o-','LineWidth',1.8);
title('Power TEG: LTC vs BQ Comparison');
xlabel('Test index'); ylabel('Mean Power [W]');
legend('LTC harvester','BQ harvester','Location','northwest');



%% ============================================================
function results = analyzeKeithleyFolder(basePath)

files = dir(fullfile(basePath,'*.csv'));

results = table('Size',[length(files) 4], ...
    'VariableTypes',{'string','double','double','double'}, ...
    'VariableNames',{'File','Period_s','UsedTime_s','MeanPower_W'});

for n = 1:length(files)

    file = fullfile(basePath, files(n).name);
    fprintf('\nProcessing %s\n', files(n).name);

    %% === Import clean (no warnings) ===
    opts = detectImportOptions(file);
    opts = setvaropts(opts, opts.VariableNames, 'Type', 'char');   % no datetime guessing
    opts.VariableNamingRule = 'preserve';

    Tk = readtable(file, opts);

    %% === Time reconstruction ===
    time = datetime(Tk.Time, 'InputFormat','HH:mm:ss');
    time = seconds(time - time(1));

    P = abs(str2double(Tk.Reading));

    %% ======================================================
    %  1) Find MPPT period (FFT)
    % ======================================================
    dt = mean(diff(time));
    Fs = 1/dt;

    P_d = P - mean(P);
    N = length(P_d);

    Y = abs(fft(P_d));
    f = (0:N-1)*(Fs/N);

    band = f > 1/17 & f < 1/14;
    [~,i] = max(Y(band));
    fband = f(band);
    f0 = fband(i);
    T = 1/f0;

    fprintf('Detected period = %.2f s\n', T);

    %% ======================================================
    %  2) Phase-aligned cut
    % ======================================================
    Ps = movmean(P, round(0.4*T/dt));
    [~,locs] = findpeaks(-Ps, 'MinPeakDistance', round(0.8*T/dt));

    if numel(locs) < 3
        t0 = time(1);
    else
        t0 = time(locs(2));
    end

    Np = floor((time(end) - t0) / T);
    usedTime = Np * T;
    t1 = t0 + usedTime;

    mask = time >= t0 & time <= t1;
    P_cut = P(mask);

    %% ======================================================
    %  3) Mean power
    % ======================================================
    Pmean = mean(P_cut);

    fprintf('Used %.1f s = %d periods\n', usedTime, Np);
    fprintf('Mean Power = %.6f W\n', Pmean);

    %% Store
    results(n,:) = {files(n).name, T, usedTime, Pmean};

end
end
