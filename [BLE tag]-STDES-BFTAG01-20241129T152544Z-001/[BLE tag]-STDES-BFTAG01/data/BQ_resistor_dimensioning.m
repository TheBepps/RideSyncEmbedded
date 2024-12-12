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


%% RESISTOR DIMENSIONING - BQ25505

V_BIAS = 1.21; 
R_SUM_D = 13e6; 
OV_D = 3.4; 
OK_HYST_D = 3.3;  
OK_PROG_D = 2.1;  % to be safe, at 1.95 we have PFET detaching VSTOR and VBAT

R_OV1 = (3/2)*(V_BIAS*R_SUM_D/OV_D);
R_OV2 = R_SUM_D - R_OV1; 

R_OK1 = V_BIAS*R_SUM_D/OK_HYST_D; 
R_OK2 = (OK_PROG_D/V_BIAS - 1)*R_OK1; 
R_OK3 = R_SUM_D - R_OK1 - R_OK2; 

% Actual commercial values 

R_OV1_c = 6.98e6; %https://www.mouser.it/ProductDetail/Vishay-Dale/CRCW06036M98FKEA?qs=IUJGSZ%2FxTQimWA9K%252BOjZ3w%3D%3D
R_OV2_c = 6.20e6;
R_OK1_c = 4.75e6; 
R_OK2_c = 3.48e6; 
R_OK3_c = 4.75e6; 

R_S_act_OV = R_OV2_c + R_OV1_c; 

OV_A = (3/2)*V_BIAS*(1 + R_OV2_c/R_OV1_c); 
OK_P_A = V_BIAS*(1 + R_OK2_c/R_OK1_c); 
OK_H_A = V_BIAS*(1 + (R_OK2_c + R_OK3_c)/R_OK1_c); 