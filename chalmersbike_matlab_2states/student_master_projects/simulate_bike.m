clc
clear all
close all


%% Parameters Initialization
% Load the configuration parameters for the simulation
configuration;


%% LQR controller
lqr_design;


%% Run the Simulink simulation
try sim(simulink_file)
        
catch Error_Reason
    Error_Reason.message,
end