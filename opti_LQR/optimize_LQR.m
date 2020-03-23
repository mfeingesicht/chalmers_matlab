clc
clear all
close all


%==========================================================================

% Features of the code and simulation
    % IMU separation from bike model implemented
    % Real/Model parameters distinction
    % 3D simulation implemented
    % Implementation of roll angle saturation for 3D simulation
    % Oscillations
    % Change of the model to consider 2 parts : frame and box NEW MODEL CORRECTED
    % Performance indicator
    
% Features NOT YET IMPLEMENTED
    % Comparison model and real bike -> statistic results on inaccurate
    % Correction of acceleration part
    % Only paths 1, 2 and 3 are correctly working

%==========================================================================

% To simulate the bike, run this script
    % Change the parameters in the script "configuration.m"
    % This script contains 7 sections:
        % Simulink
                % Path to the Simulink file to use and Simulink parameters
        % Physical bike
                % Characteristics of the bike (dimensions,mass), the sensors (noise) 
                % and the actuators (deadband, delay, limitations)
        % Model
                % Choice of the model to use (linear, nonlinear) and 
                % which kind of dynamics to include or ignore
        % LQR control
                % Definition of the LQR weight matrics (Q and R) and the
                % state-space output matrices of the system (C and D)
        % Complementary filter
                % Coefficients for the complementary filter used to
                % estimate roll
        % Path tracking
                % Choice of the path, definition of the PID coefficients,
                % definition of the matrices for the Kalman filter
                % estimating position
        % Plot settings
                % Choice of which plots to generate after the end of the
                % simulation and if an animation of the bike should be
                % generated


%==========================================================================


%% Parameters Initialization
% Load the configuration parameters for the simulation
% configuration;
% configuration_pidTuning;
configuration_lqrTuning;
% configuration_boxMovedCenter;


%% LQR controller
lqr_design;


%% Load the path
% Load all the predefined paths and select the chosen one
paths;

% Compute the cumulative distance and the curvature of the chosen path
cumulative_distance_curvature;


% %% Optimize Q ga objfun
% X0 = [diag(Q)' R];
% X0 = [6.6382    6.4971   22.1870    0.1195];
% X0 = [5.6382   10.4971   27.6870    0.1195];
% X0 = [5.8882   12.4971   32.6870    0.1195];
% 
% % % ga_options = gaoptimset('StallGenLimit',50,'Display','iter');
% % % ga_options = gaoptimset('StallGenLimit',50,'Display','iter','TimeLimit',3600);
% % ga_options = gaoptimset('StallGenLimit',50,'Display','iter','TimeLimit',3600,'InitialPopulation',X0);
% ga_options = gaoptimset('StallGenLimit',50,'Display','iter','TimeLimit',600,'InitialPopulation',X0);
% 
% lqr_coeff = ga(@objfun,length(X0),[],[],[],[],[0 0 0 0],[],[],[],ga_options)
% Q = diag(lqr_coeff(1:3));
% R = lqr_coeff(4);
% lqr_design;


%% Optimize Q ga objfun2
X0 = [diag(Q)' 0 0 0 R];
X0 = [8.6312   12.6597   19.1539    5.1451    5.8932    9.9231    0.2336];
X0 = [13.2049   21.6373   24.3988    2.6214   -6.7298   17.6896    0.2336];
X0 = [15.9549   27.6373   27.3988    5.1214   -5.7298   21.6896    0.2336];

% % ga_options = gaoptimset('StallGenLimit',50,'Display','iter');
ga_options = gaoptimset('StallGenLimit',50,'Display','iter','TimeLimit',3600);
% ga_options = gaoptimset('StallGenLimit',50,'Display','iter','TimeLimit',3600,'InitialPopulation',X0);
% ga_options = gaoptimset('StallGenLimit',50,'Display','iter','TimeLimit',600,'InitialPopulation',X0);

lqr_coeff = ga(@objfun2,length(X0),[],[],[],[],[0 0 0 -Inf -Inf -Inf 0],[],[],[],ga_options)
% lqr_coeff = X0;

Q = diag(lqr_coeff(1:3)) + squareform(lqr_coeff(4:end-1));
R = lqr_coeff(end);    
lqr_design;


%% Run the Simulink simulation
try sim(simulink_file)
        
catch Error_Reason
    Error_Reason.message,
end


%% Draw plots
% draw_plots;
draw_plots_pathComparisonReport_20190820;


%% Compute path tracking performance indicators
% performance_indicators;