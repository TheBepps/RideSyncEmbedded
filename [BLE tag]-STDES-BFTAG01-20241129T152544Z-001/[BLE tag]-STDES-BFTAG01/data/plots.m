
% setup

clc; close all; clear;

% Set LaTeX as default interpreter for axis labels, ticks and legends
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

%set(0,'DefaultFigureWindowStyle','docked');
%set(0,'DefaultFigureWindowStyle','modal');

set(0,'defaultAxesFontSize',20)
set(0,'DefaultLegendFontSize',20)

%addpath('Data')

%% SETUP 

tst_1 = table2array(readtable('test_1.csv'));
tst_2 = table2array(readtable('test_2.csv'));
tst_3 = table2array(readtable('test_3.csv'));
tst_4 = table2array(readtable('test_4.csv'));
tst_5 = table2array(readtable('test_5.csv'));
tst_6 = table2array(readtable('test_6.csv'));
tst_7 = table2array(readtable('test_7.csv'));
tst_8 = table2array(readtable('test_8.csv'));
tst_9 = table2array(readtable('test_9.csv'));
tst_10 = table2array(readtable('test_10_post.csv'));
tst_11 = table2array(readtable('test_11_v1.csv'));
tst_11_2 = table2array(readtable('test_11_v2.csv'));


T = 1; 
A = 2; 
B = 3;
C = 4; 
D = 5; 

%% TEST 1 

figure('Position', [0 0 1500 400])
plot(tst_1(:,T), tst_1(:,A), 'DisplayName', '$V_{IN}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
%plot(tst_1(:,T), tst_1(:,B), 'DisplayName', 'B', 'LineWidth', 1.5)
plot(tst_1(:,T), tst_1(:,C), 'DisplayName', '$V_{BAT,SEC}$', 'LineWidth', 1.5)
plot(tst_1(:,T), tst_1(:,D), 'DisplayName', '$V_{STOR}$', 'LineWidth', 1.5)

title('PV + plant operation, no switch')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_1(1,T) tst_1(end,T)])

%% TEST 2

figure('Position', [0 0 1500 400])
plot(tst_2(:,T), tst_2(:,A), 'DisplayName', '$V_{IN}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_2(:,T), tst_2(:,B), 'DisplayName', '$V_{BAT,OK}$', 'LineWidth', 1.5)
plot(tst_2(:,T), tst_2(:,C), 'DisplayName', '$V_{BAT,SEC}$', 'LineWidth', 1.5)

title('BQ with correct thresholds')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_2(1,T) tst_2(end,T)])

%% TEST 3

figure('Position', [0 0 1500 400])
plot(tst_3(:,T), tst_3(:,A), 'DisplayName', '$V_{IN}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_3(:,T), tst_3(:,B), 'DisplayName', '$V_{DD}$', 'LineWidth', 1.5)
plot(tst_3(:,T), tst_3(:,C), 'DisplayName', '$V_{BAT,SEC}$', 'LineWidth', 1.5)
plot(tst_3(:,T), tst_3(:,D), 'DisplayName', 'TXE', 'LineWidth', 1.5)

title('Full system: plant, switch, covered PV')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_3(1,T) 60])
ylim([-0.1 3.5])


%% TEST 4

% THIS TIME IT SWITCHED AT 2.45 V: probably something wrong with
% measurements

% figure('Position', [0 0 1500 400])
% plot(tst_4(:,T), tst_4(:,A), 'DisplayName', '$V_{DD}$', 'LineWidth', 1.5)
% box on
% grid on 
% hold on 
% plot(tst_4(:,T), tst_4(:,B), 'DisplayName', 'TXE', 'LineWidth', 1.5)
% 
% title('TAG working on its own: no BQ, no plant, uncovered PV')
% legend
% xlabel('Time [s]')
% ylabel('Voltage [V]')
% xlim([tst_4(1,T) tst_4(end,T)])

%% TEST 5

% Test 4 repeated: now it switches at correct voltages

figure('Position', [0 0 1500 400])
subplot(2,1,1)
plot(tst_5(:,T), tst_5(:,A), 'DisplayName', '$V_{DD}$', 'LineWidth', 1.5)
box on
grid on 
hold on 

title('TAG working on its own: no BQ, no plant, uncovered PV')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_5(1,T) tst_5(end,T)])

subplot(2,1,2)
box on
grid on 
hold on 
plot(tst_5(:,T), tst_5(:,B), 'DisplayName', 'TXE', 'LineWidth', 1.5)
plot(tst_5(:,T), tst_5(:,C), 'DisplayName', 'BTH', 'LineWidth', 1.5)
plot(tst_5(:,T), tst_5(:,D), 'DisplayName', 'WFE', 'LineWidth', 1.5)
xlabel('Time [s]')
ylabel('Voltage [V]')
legend
xlim([tst_5(1,T) tst_5(end,T)])

%% TEST 6

% test_5 but zoomed on a single beacon 
figure('Position', [0 0 1500 400])
subplot(2,1,1)
plot(tst_6(:,T), tst_6(:,A), 'DisplayName', '$V_{DD}$', 'LineWidth', 1.5)
box on
grid on 
hold on 

title('TAG working on its own: no BQ, no plant, uncovered PV')
legend
xlabel('Time [ms]')
ylabel('Voltage [V]')
xlim([tst_6(1,T) 40])

subplot(2,1,2)
box on
grid on 
hold on 
plot(tst_6(:,T), tst_6(:,B), 'DisplayName', 'TXE', 'LineWidth', 1.5)
plot(tst_6(:,T), tst_6(:,C), 'DisplayName', 'BTH', 'LineWidth', 1.5)
plot(tst_6(:,T), tst_6(:,D), 'DisplayName', 'WFE', 'LineWidth', 1.5)
xlabel('Time [ms]')
ylabel('Voltage [V]')
legend
xlim([tst_6(1,T) 40])

% COMPUTE ENERGY FOR SINGLE BEACON

V_init = mean(tst_6(1:1000,A)); 
V_end = 2.07; 

EN_TX = 0.5*440e-6*(V_init^2 - V_end^2);

%% TEST 7

% figure('Position', [0 0 1500 400])
% plot(tst_7(:,T), tst_7(:,A), 'DisplayName', 'TXE', 'LineWidth', 1.5)
% box on
% grid on 
% hold on 
% plot(tst_7(:,T), tst_7(:,B), 'DisplayName', '$V_{DD}$', 'LineWidth', 1.5)
% plot(tst_7(:,T), tst_7(:,C), 'DisplayName', '$V_{IN}$', 'LineWidth', 1.5)
% plot(tst_7(:,T), tst_7(:,D), 'DisplayName', '$V_{BAT,OK}$', 'LineWidth', 1.5)
% 
% title('Full system: simulated plant, switch, covered PV')
% legend
% xlabel('Time [s]')
% ylabel('Voltage [V]')
% xlim([tst_7(1,T) tst_7(end,T)])
% ylim([-0.1 3.5])

%% TEST 8 

figure('Position', [0 0 1500 400])
plot(tst_8(:,T), tst_8(:,A), 'DisplayName', '$V_{OUT}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_8(:,T), tst_8(:,B), 'DisplayName', '$V_{IN} = V_{STOR}$', 'LineWidth', 1.5)
plot(tst_8(:,T), tst_8(:,C), 'DisplayName', '$V_{EN} = V_{BAT,OK}$', 'LineWidth', 1.5)
plot(tst_8(:,T), tst_8(:,D), 'DisplayName', '$V_{BAT,SEC}$', 'LineWidth', 1.5)

title('Incomplete system: simulated plant, switch, no load')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_8(1,T) tst_8(end,T)])
ylim([-0.1 3.5])

%% TEST 9

figure('Position', [0 0 1500 400])
plot(tst_9(:,T), tst_9(:,A), 'DisplayName', '$V_{OUT}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_9(:,T), tst_9(:,B), 'DisplayName', '$V_{IN} = V_{STOR}$', 'LineWidth', 1.5)
plot(tst_9(:,T), tst_9(:,C), 'DisplayName', '$V_{EN} = V_{BAT,OK}$', 'LineWidth', 1.5)
plot(tst_9(:,T), tst_9(:,D), 'DisplayName', 'TXE', 'LineWidth', 1.5)

title('Full system: simulated plant, switch, BFTAG attached to switch output, covered PV')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_9(1,T) tst_9(end,T)])
ylim([-0.1 3.5])

%% TEST 10

figure('Position', [0 0 1500 400])
plot(tst_10(:,T), tst_10(:,A), 'DisplayName', '$V_{OUT,SW} = V_{DD}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_10(:,T), tst_10(:,B), 'DisplayName', '$V_{IN,SW} = V_{STOR}$', 'LineWidth', 1.5)
plot(tst_10(:,T), tst_10(:,C), 'DisplayName', '$V_{IN, PLANT}$', 'LineWidth', 1.5)
plot(tst_10(:,T), tst_10(:,D), 'DisplayName', 'TXE', 'LineWidth', 1.5)

title('Full system: real plant, switch, BFTAG attached to switch output, covered PV')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_10(1,T) tst_10(end,T)])
ylim([-0.1 3.5])

%% TEST 11

figure('Position', [0 0 1500 400])
plot(tst_11(:,T), tst_11(:,A), 'DisplayName', '$V_{OUT,SW} = V_{DD}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_11(:,T), tst_11(:,B), 'DisplayName', '$V_{IN,SW} = V_{STOR}$', 'LineWidth', 1.5)
plot(tst_11(:,T), tst_11(:,C), 'DisplayName', '$V_{IN, PLANT}$', 'LineWidth', 1.5)
plot(tst_11(:,T), tst_11(:,D), 'DisplayName', 'TXE', 'LineWidth', 1.5)

title('Full system: STRESS TEST 1, real plant, switch, BFTAG attached to switch output, covered PV')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_11(1,T) tst_11(end,T)])
ylim([-0.1 3.5])

figure('Position', [0 0 1500 400])
plot(tst_11_2(:,T), tst_11_2(:,A), 'DisplayName', '$V_{OUT,SW} = V_{DD}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_11_2(:,T), tst_11_2(:,B), 'DisplayName', '$V_{IN,SW} = V_{STOR}$', 'LineWidth', 1.5)
plot(tst_11_2(:,T), tst_11_2(:,C), 'DisplayName', '$V_{IN, PLANT}$', 'LineWidth', 1.5)
plot(tst_11_2(:,T), tst_11_2(:,D), 'DisplayName', 'TXE', 'LineWidth', 1.5)

title('Full system: STRESS TEST 2, real plant, switch, BFTAG attached to switch output, covered PV')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_11_2(1,T) tst_11_2(end,T)])
ylim([-0.1 3.5])

