% Parameters
P_TLV   = 248e-9;       % TLV comparator power [W]
Rds_on  = 178e-3;       % On-resistance of SIP [Ohm]
P_AND4  = 231e-9;        % Power per group of 4 AND gates [W]
P_NAND4 = 231e-9;        % Power per group of 4 NAND gates [W]

% Sweep ranges
n_vals = linspace(3,8.9,100);                         % number of inputs
%I_vals = logspace(-4, -2, 200);       % current from 100 uA to 10 mA, finer grid
I_vals = linspace(100e-6, 5e-3, 50);

% Create meshgrid
[N, I] = meshgrid(n_vals, I_vals);

%% Configuration A: SIP + TLV
P_SIP = (I.^2) * Rds_on;                       % power per SIP
P_tot_A_w = (floor(N) - 1) .* (P_TLV + P_SIP);           % power worst case
P_tot_A_b = (floor(N) - 1) .* (P_TLV)  + P_SIP;          % power best case
P_tot_A = (0.6*P_tot_A_b+0.4*P_tot_A_w);


%% Configuration B: SIP + TLV + Logic
% Numero intero da usare per i gruppi logici
N_int = floor(N);          % o round(N) se vuoi arrotondare al più vicino intero

% Numero dispositivi logici richiesti (arrotondati per eccesso)
logic_groups_NAND = ceil((N_int .* (N_int-1) ./ 2) ./ 4);   % NAND a gruppi di 4
logic_groups_AND  = ceil((N_int .* (N_int-2)) ./ 4);        % AND a gruppi di 4

% Potenza totale
P_tot_B = P_SIP ...
         + (N_int .* (N_int-1) ./ 2) .* P_TLV ...   % continua per P_TLV
         + logic_groups_NAND .* P_NAND4 ... % gradino per NAND
         + logic_groups_AND  .* P_AND4;     % gradino per AND


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
grid on; view(-45,30);
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
grid on; view(-45,30);
xticks(3:1:8)   % Mostra solo 3,4,5,6,7,8


%% Figure 3: Surfaces con gradiente colorato corretto (n discreto)
figure; hold on;

% Normalizziamo i valori tra 0 e 1
C_A = (P_tot_A - min(P_tot_A(:))) / (max(P_tot_A(:)) - min(P_tot_A(:)));
C_B = (P_tot_B - min(P_tot_B(:))) / (max(P_tot_B(:)) - min(P_tot_B(:)));

% Config A: celeste chiaro -> blu intenso
%RGB_A = cat(3, 0.4 + 0.1*C_A, 0.8 - 0.3*C_A, 1 - 0.2*C_A);  
RGB_A = cat(3, 0.8*C_A + 0.2, 0.5*C_A + 0.2, 1 - 0.5*C_A);

%RGB_A = cat(3, (1-C_A), (1-C_A), (1-C_A));

% Config B: Rosso intenso -> Verde puro
RGB_B = cat(3, 1 - C_B, C_B, zeros(size(C_B)));  

% Plot superfici
hA = surf(N, I*1e6, P_tot_A*1e6, RGB_A, ...
          'FaceAlpha',0.7, 'EdgeColor','none', 'FaceColor','interp');
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

% Aggiungi linee verticali lungo la corrente per N interi
n_int = 3:8;   % valori interi di N
for k = 1:length(n_int)
    % Trova il primo indice in cui n_vals è maggiore o uguale a n_int(k)
    idx = find(n_vals >= n_int(k), 1, 'first');
    
    % Estrai la colonna di potenza per N intero
    P_A_col = P_tot_A(:, idx);
    P_B_col = P_tot_B(:, idx);
    
    % Traccia le linee 3D
    plot3(n_vals(idx)*ones(size(I)), I*1e6, P_A_col*1e6, 'k', 'LineWidth', 1.5);
    plot3(n_vals(idx)*ones(size(I)), I*1e6, P_B_col*1e6, 'k', 'LineWidth', 1.5);
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

grid on; view(-45,30);
hold off;


%% calcolo interserioni
P_logic_unit = P_AND4 + P_NAND4;
n = 3:8;    % valori discreti di N (input count)
I_vals = zeros(size(n));

for k = 1:length(n)    
    % Numero di gruppi logici per questo valore di n
    logic_groups = floor((n(k) - 1) / 4) + 1;

    % Potenza logica
    P_logic = logic_groups * P_logic_unit;
    
    % Intersezione: P_tot_A = P_tot_B  → ricavo la corrente I
    I_vals(k) = sqrt( (P_TLV + P_logic) / ((n(k)-2) * Rds_on) );
end

% Mostra tabella con correnti
result = table(n.', I_vals.', I_vals.'*1e3, I_vals.'*1e6, ...
      'VariableNames', {'n','I_A','I_mA','I_uA'})



