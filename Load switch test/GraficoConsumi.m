clear; clc; close all;

% =========================================================================
% UPDATED PARAMETERS (SC-70 / 5V Operation)
% =========================================================================
P_TLV   = 375e-9;        % TLV comparator power [W] (Typ @ 5V)
Rds_on  = 147e-3;        % On-resistance of SIP [Ohm] (Typ @ 5V)
P_AND4  = 350e-9;        % Power per group of 4 AND gates [W] (Typ @ 5V)
P_NAND4 = 350e-9;        % Power per group of 4 NAND gates [W] (Typ @ 5V)

% Sweep ranges
n_vals = linspace(3,8.9,100);             % number of inputs
I_vals = linspace(100e-6, 5e-3, 50);      % current from 100 uA to 5 mA

% Create meshgrid
[N, I] = meshgrid(n_vals, I_vals);

%% Configuration A: SIP + TLV + Logic (Logic-Based)
% Note: Nel testo Config A è quella CON logica esterna.
% Regola gruppi logici: 1 gruppo per n=3, 2 gruppi per n>=4
logic_groups = 1 .* (floor(N) < 4) + 2 .* (floor(N) >= 4);

% Potenza Statica Logica
P_logic_total = logic_groups .* (P_AND4 + P_NAND4);

% P_A = n*TLV + Logic + Single SIP (Best case conduction)
P_tot_A = (floor(N) .* P_TLV) + P_logic_total + (I.^2 .* Rds_on);

%% Configuration B: SIP + TLV (Logic-less)
% Note: Nel testo Config B è quella SENZA logica (Daisy Chain).
% P_B = (n-1)*TLV + (n-1)*SIP (Worst case conduction)
% Usiamo (n-1) switch in serie nel caso peggiore
P_tot_B = ((floor(N) - 1) .* P_TLV) + ((floor(N) - 1) .* I.^2 .* Rds_on);

%% Figure 1: Configuration A (Logic-Based)
figure;
hA = surf(N, I*1e6, P_tot_A*1e6, 'FaceAlpha',0.8, 'EdgeColor','k');
colormap('parula');
xlabel('Number of Inputs (n)');
ylabel('Input Current I [\muA]');
zlabel('Power Dissipation [\muW]');
title('Configuration A: SIP + TLV + Logic');
legend(hA,'Config A (Logic-Based)','Location','best');
grid on; view(-45,30);
xticks(3:1:8)

%% Figure 2: Configuration B (Logic-Less)
figure;
hB = surf(N, I*1e6, P_tot_B*1e6, 'FaceAlpha',0.8, 'EdgeColor','k');
colormap('default');
xlabel('Number of Inputs (n)');
ylabel('Input Current I [\muA]');
zlabel('Power Dissipation [\muW]');
title('Configuration B: SIP + TLV (Logic-less)');
legend(hB,'Config B (Logic-Less)','Location','best');
grid on; view(-45,30);
xticks(3:1:8)

%% Figure 3: Comparison and Intersection
figure; hold on;

% Normalizziamo i valori tra 0 e 1 per i colori personalizzati
% Calcoliamo min e max globali per una scala coerente
all_P = [P_tot_A(:); P_tot_B(:)];
min_P = min(all_P); max_P = max(all_P);

C_A = (P_tot_A - min_P) / (max_P - min_P);
C_B = (P_tot_B - min_P) / (max_P - min_P);

% Config A (Logic): Blu/Celeste (Consumo alto statico)
RGB_A = cat(3, 0.2*ones(size(C_A)), 0.6*C_A+0.2, 0.9*ones(size(C_A)));

% Config B (Logic-less): Rosso/Arancio (Basso statico, sale con corrente)
RGB_B = cat(3, 1.0*ones(size(C_B)), 0.5*C_B, 0.2*ones(size(C_B)));

% Plot superfici
hA = surf(N, I*1e6, P_tot_A*1e6, RGB_A, ...
          'FaceAlpha',0.7, 'EdgeColor','none', 'FaceColor','interp');
hB = surf(N, I*1e6, P_tot_B*1e6, RGB_B, ...
          'FaceAlpha',0.6, 'EdgeColor','none', 'FaceColor','interp');

% Intersection points calculation (Numeric scan on mesh)
hInt = [];
for ni = 1:length(n_vals)
    diffP = (P_tot_A(:,ni) - P_tot_B(:,ni));
    % Trova dove la differenza cambia segno
    sign_change_idx = find(diffP(1:end-1).*diffP(2:end) <= 0);
    
    for k = sign_change_idx'
        I_int = (I(k,ni) + I(k+1,ni)) / 2;
        P_int = (P_tot_A(k,ni) + P_tot_B(k,ni))/2; 
        % Plot solo punti validi
        hInt = scatter3(n_vals(ni), I_int*1e6, P_int*1e6, 20, 'k', 'filled');
    end
end

% Aggiungi linee verticali lungo la corrente per N interi
n_int = 3:8; 
for k = 1:length(n_int)
    idx = find(n_vals >= n_int(k), 1, 'first');
    P_A_col = P_tot_A(:, idx);
    P_B_col = P_tot_B(:, idx);
    plot3(n_vals(idx)*ones(size(I(:,1))), I(:,1)*1e6, P_A_col*1e6, 'k', 'LineWidth', 1.0);
    plot3(n_vals(idx)*ones(size(I(:,1))), I(:,1)*1e6, P_B_col*1e6, 'k', 'LineWidth', 1.0);
end

xticks(3:1:8)
xlabel('Number of Inputs (n)');
ylabel('Input Current I [\muA]');
zlabel('Power Dissipation [\muW]');
title('Comparison: Logic-Based (A) vs Logic-Less (B)');
legend([hA, hB, hInt], ...
       {'Config A (Logic-Based)','Config B (Logic-Less)','Intersection'}, ...
       'Location','best');
grid on; view(135,25); % Vista ruotata per vedere meglio l'intersezione
hold off;

%% Calcolo Intersezioni Analitiche (Tabella Finale)
P_logic_unit = P_AND4 + P_NAND4;
n_discrete = 3:8;    
I_thresholds = zeros(size(n_discrete));

fprintf('\n--- Intersection Analysis Results ---\n');
fprintf('%-5s | %-10s | %-10s\n', 'n', 'I_th [mA]', 'I_th [uA]');

for k = 1:length(n_discrete)    
    % Numero di gruppi logici
    if n_discrete(k) == 3
        lg = 1;
    else
        lg = 2;
    end
    
    P_logic = lg * P_logic_unit;
    
    % Formula inversa dal testo LaTeX:
    % I = sqrt( (P_TLV + P_logic) / ((n-2)*Rds) )
    numerator = P_TLV + P_logic;
    denominator = (n_discrete(k) - 2) * Rds_on;
    
    I_val = sqrt( numerator / denominator );
    I_thresholds(k) = I_val;
    
    fprintf('%-5d | %-10.3f | %-10.0f\n', n_discrete(k), I_val*1e3, I_val*1e6);
end

% Salva risultati in tabella workspace
IntersectionTable = table(n_discrete.', I_thresholds.'*1e3, I_thresholds.'*1e6, ...
      'VariableNames', {'n','I_th_mA','I_th_uA'});

%% Figure 4: 2D Cross-Section for n = 4 Inputs
figure('Name', '2D Cross-Section n=4');
n_target = 4;

% Trova l'indice nel vettore n_vals più vicino a 4
[~, idx_n] = min(abs(n_vals - n_target));

% Estrai i vettori di potenza per quell'indice
P_A_slice = P_tot_A(:, idx_n);
P_B_slice = P_tot_B(:, idx_n);

% Plot
plot(I_vals*1e3, P_A_slice*1e6, 'b-', 'LineWidth', 2); hold on;
plot(I_vals*1e3, P_B_slice*1e6, 'r--', 'LineWidth', 2);

% Trova intersezione analitica per n=4
logic_groups_4 = 2; % per n=4 servono 2 gruppi
P_logic_4 = logic_groups_4 * P_AND4 + P_NAND4; % O quello che hai definito
% Ricalcola soglia precisa
num = P_TLV + (logic_groups_4 * (P_AND4+P_NAND4));
den = (n_target - 2) * Rds_on;
I_th_4 = sqrt(num/den);

% Segna il punto di intersezione
xline(I_th_4*1e3, 'k:', 'LineWidth', 1.5, 'Label', sprintf('  I_{th} = %.2f mA', I_th_4*1e3));
scatter(I_th_4*1e3, (n_target*P_TLV + (logic_groups_4*(P_AND4+P_NAND4)) + I_th_4^2*Rds_on)*1e6, 50, 'k', 'filled');

xlabel('Load Current I [mA]');
ylabel('Power Dissipation [\muW]');
title(sprintf('Power Comparison for Fixed Inputs (n=%d)', n_target));
legend('Config A (Logic)', 'Config B (Logic-less)', 'Threshold');
grid on;