% Import CSV file, skipping the first 2 header rows (names + units)
T = readtable('Test SIP+TLV\20250918-loop_condition_sip+tlv.csv', 'NumHeaderLines', 2, 'Delimiter', ',');

% Check the actual variable names (MATLAB replaces spaces with underscores)
disp(T.Properties.VariableNames);

% Assign columns to variables with clear names
Time      = T.("Var1");
Channel_A = T.("Var2");
Channel_B = T.("Var3");
Channel_C = T.("Var4");
Channel_D = T.("Var5");

% Smooth the channels using a moving average (window = x points)
Window = 10;
Channel_A_smooth = smooth(Channel_A, Window);
Channel_B_smooth = smooth(Channel_B, Window);  
Channel_C_smooth = smooth(Channel_C, Window);
Channel_D_smooth = smooth(Channel_D, Window);

% Create plot
figure;
plot(Time, Channel_A, '-g', 'LineWidth', 1.5); hold on;
plot(Time, Channel_B, '-r', 'LineWidth', 1.5); 
plot(Time, Channel_C, '-b', 'LineWidth', 1.5);
plot(Time, Channel_D, '-k', 'LineWidth', 1.5);
hold off;

% Plot smoothed data
figure;
plot(Time, Channel_A_smooth, '-g', 'LineWidth', 1.5); hold on;
plot(Time, Channel_B_smooth, '-r', 'LineWidth', 1.5);
plot(Time, Channel_C_smooth, '-b', 'LineWidth', 1.5);
plot(Time, Channel_D_smooth, '-k', 'LineWidth', 1.5);
hold off;


xlim([-1 4]);
ylim([0 3.5]);

% Formatting
grid on;
xlabel('Time [ms]');        % You can manually add units if needed
ylabel('Amplitude [V]');   % Or adapt this label
title('SiP+TLV loop condition'); 
legend('Vout','Vin1','Vin2','Enable', 'Location', 'best');
