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
configuration_pidTuning;


%% LQR controller
lqr_design;
    

%% Load the path
% Load all the predefined paths and select the chosen one
paths;

% Compute the cumulative distance and the curvature of the chosen path
cumulative_distance_curvature;


%% Create list of PID gains to test
%
% K_PID = [K_P K_I K_D]
%
% 
% 
%                [ K_P_1 K_I_1 K_D_1 ]
% K_comparison = [ K_P_2 K_I_2 K_D_2 ]
%                [ K_P_3 K_I_3 K_D_3 ]
% 

% % K_comparison = [0.15 0 0];
% % K_comparison = [0.15 0.1 0];
% % K_comparison = [0.6 0 0 ; 1.7 0 0];
% % K_comparison = [0.6 0 0 ; 0.6 0.1 0];
% K_comparison = [0.25 0.1 0.3];
% K_comparison = [0.35 0 0.5];


K_comparison = [0.02 0 0];


% % 'fast' LQR
% K_dir = [0.2 0 0];
% K_pos = [0.4 0 0];

% % 'medium' LQR
% K_dir = [0.4 0 0];
% K_pos = [0.185 0 0];

% 'slow' LQR
K_dir = [0.2 0 0];
K_pos = [0.185 0 0];


%% Create vectors to store the simulation results
time = cell(size(K_comparison,1),1);
roll_comparison = cell(size(K_comparison,1),1);
steering_comparison = cell(size(K_comparison,1),1);
rollrate_comparison = cell(size(K_comparison,1),1);
stepinput_comparison = cell(size(K_comparison,1),1);
stepoutput_comparison = cell(size(K_comparison,1),1);
X_BIKE_comparison = cell(size(K_comparison,1),1);
Y_BIKE_comparison = cell(size(K_comparison,1),1);
X_SIGMA_comparison = cell(size(K_comparison,1),1);
Y_SIGMA_comparison = cell(size(K_comparison,1),1);
heading_comparison = cell(size(K_comparison,1),1);


%% Run simulation for each set of weights
for index=1:size(K_comparison,1)
    K_PID = K_comparison(index,:);


    %% Run the Simulink simulation
    try sim(simulink_file)

    catch Error_Reason
        Error_Reason.message,
    end

    time{index} = states_true.time;
    roll_comparison{index} = rad2deg(states_true.data(:,1));
    steering_comparison{index} = rad2deg(states_true.data(:,2));
    rollrate_comparison{index} = rad2deg(states_true.data(:,3));
    stepinput_comparison{index} = rad2deg(stepinput.data);
    stepoutput_comparison{index} = rad2deg(stepoutput.data);
    X_BIKE_comparison{index} = X_BIKE.data;
    Y_BIKE_comparison{index} = Y_BIKE.data;
    X_SIGMA_comparison{index} = X_SIGMA.data;
    Y_SIGMA_comparison{index} = Y_SIGMA.data;
    heading_comparison{index} = heading.data;
end


%% Plot all simulations together
draw_plots_pid_testing;
figure;hold on;plot(stepinput.Time,rad2deg(stepinput.Data));plot(stepoutput.Time,rad2deg(stepoutput.Data));title('Heading Step Response');legend('Heading Step Reference','Bike Heading');xlabel('Time [s]');ylabel('Heading [deg]');
% figure;hold on;plot(heading_estim);plot(heading_estim_deltaBias);plot(heading_estim_deltaBias_linear)
% draw_plots_pathComparisonReport_20190820;
% figure;hold on;plot(stepinput);plot(stepoutput);
% figure;hold on;plot(heading.Time,rad2deg(heading.Data));plot(heading_estim.Time,rad2deg(heading_estim.Data));plot(heading_estim_deltaBias.Time,rad2deg(heading_estim_deltaBias.Data));