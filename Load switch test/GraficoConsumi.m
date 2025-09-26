% Parameters
P_TLV   = 248e-9;       % TLV comparator power [W]
Rds_on  = 178e-3;       % On-resistance of SIP [Ohm]
P_AND4  = 231e-9;        % Power per group of 4 AND gates [W]
P_NAND4 = 231e-9;        % Power per group of 4 NAND gates [W]

% Sweep ranges
n_vals = linspace(3,8.9,100);                         % number of inputs
n_vals_disc = 3:8.9;
%I_vals = logspace(-4, -2, 200);       % current from 100 uA to 10 mA, finer grid
I_vals = linspace(100e-6, 5e-3, 50);

% Create meshgrid
[N, I] = meshgrid(n_vals, I_vals);
[N_disc, I_disc] = meshgrid(n_vals_disc, I_vals);

%% Configuration A: SIP + TLV
P_SIP = (I.^2) * Rds_on;                       % power per SIP
P_SIP_disc = (I_disc.^2) * Rds_on;
P_tot_A = (floor(N) - 1) .* (P_TLV + P_SIP);          % total power
P_tot_A_disc = (N_disc - 1) .* (P_TLV + P_SIP_disc);

%% Configuration B: SIP + TLV + Logic
logic_groups = ones(size(N));     % inizializzi tutto a 1
logic_groups(N >= 4) = 2;         % per N >= 4 assegni 2
logic_groups_disc = ceil(N_disc ./ 4);
P_logic = logic_groups .* (P_AND4 + P_NAND4);  % total logic contribution
P_logic_disc = logic_groups_disc .* (P_AND4 + P_NAND4);
P_tot_B = floor(N) .* P_TLV + P_logic + P_SIP;        % total power
P_tot_B_disc = N_disc .* P_TLV + P_logic_disc + P_SIP_disc;

%% Figure 1: Configuration A
figure;
hA = surf(N, I*1e6, P_tot_A*1e6, 'FaceAlpha',0.8, 'EdgeColor','k');
%shading flat;
colormap('parula');
xlabel('Number of Inputs (n)');
ylabel('Input Current I [\muA]');
zlabel('Power Dissipation [\muW]');
title('Configuration A: SIP + TLV');
legend(hA,'Config A (SIP+TLV)','Location','best');
grid on; view(45,30);
xticks(3:1:8)   % Mostra solo 3,4,5,6,7,8

%% Figure 2: Configuration B
figure;
hB = surf(N, I*1e6, P_tot_B*1e6, 'FaceAlpha',0.8, 'EdgeColor','k');
%shading flat;
colormap('default');
xlabel('Number of Inputs (n)');
ylabel('Input Current I [\muA]');
zlabel('Power Dissipation [\muW]');
title('Configuration B: SIP + TLV + Logic');
legend(hB,'Config B (SIP+TLV+Logic)','Location','best');
grid on; view(45,30);
xticks(3:1:8)   % Mostra solo 3,4,5,6,7,8


%% Figure 3: Surfaces con gradiente colorato corretto (n discreto)
figure; hold on;

% Normalizziamo i valori tra 0 e 1
C_A = (P_tot_A - min(P_tot_A(:))) / (max(P_tot_A(:)) - min(P_tot_A(:)));
C_B = (P_tot_B - min(P_tot_B(:))) / (max(P_tot_B(:)) - min(P_tot_B(:)));

% Config A: celeste chiaro -> blu intenso
RGB_A = cat(3, 0.4 + 0.1*C_A, 0.8 - 0.3*C_A, 1 - 0.2*C_A);  

% Config B: giallo chiaro -> rosso intenso
RGB_B = cat(3, ones(size(C_B)), 1 - 0.5*C_B, 0.2 + 0.8*C_B);  

% Plot superfici
hA = surf(N, I*1e6, P_tot_A*1e6, RGB_A, ...
          'FaceAlpha',0.6, 'EdgeColor','none', 'FaceColor','interp');
hB = surf(N, I*1e6, P_tot_B*1e6, RGB_B, ...
          'FaceAlpha',0.6, 'EdgeColor','none', 'FaceColor','interp');

% Intersection points
hInt = [];
for ni = 1:length(n_vals)
    diffP = (P_tot_A(:,ni) - P_tot_B(:,ni));
    sign_change_idx = find(diffP(1:end-1).*diffP(2:end) < 0);
    for k = sign_change_idx'
        I_int = (I(k,ni) + I(k+1,ni)) / 2;
        P_int = (P_tot_A(k,ni)+P_tot_A(k+1,ni) + P_tot_B(k,ni)+P_tot_B(k+1,ni))/4;
        hInt = scatter3(n_vals(ni), I_int*1e6, P_int*1e6, 10, 'k', 'filled');
    end
end

xticks(3:1:8)   % Mostra solo 3,4,5,6,7,8
xlabel('Number of Inputs (n)');
ylabel('Input Current I [\muA]');
zlabel('Power Dissipation [\muW]');
title('Comparison and Intersection of Configurations A and B');

% Legenda
legend([hA, hB, hInt], ...
       {'Config A (SIP+TLV)','Config B (SIP+TLV+Logic)','Intersection'}, ...
       'Location','best');

grid on; view(45,30);
hold off;


%% calcolo interserioni
P_TLV   = 248e-9;
Rds_on  = 178e-3;
P_AND4  = 231e-9;
P_NAND4 = 231e-9;
P_logic_unit = P_AND4 + P_NAND4;

n = 3:8;
I_vals = zeros(size(n));
for k = 1:length(n)
    if n(k) == 3
        logic_groups = 1;
    else
        logic_groups = 2;
    end
    P_logic = logic_groups * P_logic_unit;
    I_vals(k) = sqrt( (P_TLV + P_logic) / ((n(k)-2)*Rds_on) );
end

table(n.', I_vals.', I_vals.'*1e3, I_vals.'*1e6, ...
      'VariableNames', {'n','I_A','I_mA','I_uA'})


