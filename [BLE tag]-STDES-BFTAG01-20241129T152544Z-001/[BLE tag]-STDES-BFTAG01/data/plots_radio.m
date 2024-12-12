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

tst_12 = table2array(readtable('test_12.csv'));
tst_13 = table2array(readtable('test_13.csv'));
tst_14 = table2array(readtable('test_14.csv'));
tst_15 = table2array(readtable('test_15.csv'));
tst_16 = table2array(readtable('test_16.csv'));
tst_17 = table2array(readtable('test_17.csv'));


T = 1; 
A = 2; 
B = 3;
C = 4; 
D = 5; 

%% TEST 12

figure('Position', [0 0 1500 400])
subplot(2,2,1)
plot(tst_12(:,T), tst_12(:,A), 'DisplayName', '$V_{IN}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_12(:,T), tst_12(:,B), 'DisplayName', '$TXE$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,C), 'DisplayName', '$V_{BAT,SEC}$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,D), 'DisplayName', '$V_{STOR}$', 'LineWidth', 1.5)

title('BFTAG - normal operation')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_12(1,T) tst_12(end,T)])

%% TEST 13

%figure('Position', [0 0 1500 400])
subplot(2,2,2)
plot(tst_13(:,T), tst_13(:,A), 'DisplayName', '$V_{IN}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_13(:,T), tst_13(:,B), 'DisplayName', '$TXE$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,C), 'DisplayName', '$V_{BAT,SEC}$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,D), 'DisplayName', '$V_{STOR}$', 'LineWidth', 1.5)

title('BFTAG - $PWR\_PVDLEVEL\_5$')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_13(1,T) tst_13(end,T)])

%% TEST 14

%figure('Position', [0 0 1500 400])
subplot(2,2,3)
plot(tst_14(:,T), tst_14(:,A), 'DisplayName', '$V_{IN}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_14(:,T), tst_14(:,B), 'DisplayName', '$TXE$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,C), 'DisplayName', '$V_{BAT,SEC}$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,D), 'DisplayName', '$V_{STOR}$', 'LineWidth', 1.5)

title('BFTAG - $PWR\_PVDLEVEL\_2$')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_14(1,T) tst_14(end,T)])

%% TEST 15

%figure('Position', [0 0 1500 400])
subplot(2,2,4)
plot(tst_15(:,T), tst_15(:,A), 'DisplayName', '$V_{IN}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_15(:,T), tst_15(:,B), 'DisplayName', '$TXE$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,C), 'DisplayName', '$V_{BAT,SEC}$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,D), 'DisplayName', '$V_{STOR}$', 'LineWidth', 1.5)

title('BFTAG - $PWR\_PVDLEVEL\_0$')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_15(1,T) tst_15(end,T)])

%% TEST 16

figure('Position', [0 0 1500 400])
%subplot(2,2,4)
plot(tst_16(:,T), tst_16(:,A), 'DisplayName', '$V_{IN}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_16(:,T), tst_16(:,B), 'DisplayName', '$TXE$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,C), 'DisplayName', '$V_{BAT,SEC}$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,D), 'DisplayName', '$V_{STOR}$', 'LineWidth', 1.5)

title('BFTAG - $PWR\_PVDLEVEL\_4$')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_16(1,T) tst_16(end,T)])

%% TEST 17

figure('Position', [0 0 1500 400])
%subplot(2,2,4)
plot(tst_17(:,T), tst_17(:,A), 'DisplayName', '$V_{IN}$', 'LineWidth', 1.5)
box on
grid on 
hold on 
plot(tst_17(:,T), tst_17(:,B), 'DisplayName', '$TXE$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,C), 'DisplayName', '$V_{BAT,SEC}$', 'LineWidth', 1.5)
%plot(tst_12(:,T), tst_12(:,D), 'DisplayName', '$V_{STOR}$', 'LineWidth', 1.5)

title('BFTAG01 - autonomous BLE sensor node')
legend
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([tst_17(1,T) tst_17(end,T)])