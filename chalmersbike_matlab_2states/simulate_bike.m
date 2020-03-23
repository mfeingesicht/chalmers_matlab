% clc
clear all
% close all


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
configuration;
% configuration_pidTuning;
% configuration_boxMovedCenter;


%% Test
Q_phi = [10 -9 ; -9 100];
R_phi = 1;

P_steering_position = 30;
I_steering_position = 30;


%% LQR controller
lqr_design;
lqr_gains,


%% Prediction for steering motor delay compensation
% Prediction closed-loop state space matrices
if d_delay>0
    L = lqr_gains(2,2:end);
    Abar = [     A_bike_dis          ,           zeros(n,n*d_delay)                      ,      zeros(n,d_delay-1)        ,      B_bike_dis        ,       zeros(n,d_delay-1)       ,       zeros(n,1)      ;
                             eye(n*d_delay)                 zeros(n*d_delay,n)           ,   zeros(n*d_delay,d_delay-1)   ,   zeros(n*d_delay,1)   ,   zeros(n*d_delay,d_delay-1)   ,   zeros(n*d_delay,1)  ;
            L-L*A_bike_dis^d_delay   ,   zeros(1,n*(d_delay-1)) -L*A_bike_dis^d_delay    ,                      -L * vec_powers_AB                 ,                      -L * vec_powers_AB                ;
             zeros(2*d_delay-1,n)    ,        zeros(2*d_delay-1,n*d_delay)               ,                       eye(2*d_delay-1)                  ,                     zeros(2*d_delay-1,1)               ];
    Bbar = [zeros(n*(d_delay+1),1) ; 1 ; zeros(2*d_delay-1,1)];
    Cbar = [C1_bike_dis*input_velocity C2_bike_dis*input_velocity^2];
    Cbar = [Cbar zeros(1,size(Abar,2)-2)];
    sysbar = ss(Abar,Bbar,Cbar,0,Ts);
    figure;bode(sysbar)
    
    if all(eig(Abar)<1)
        disp('The closed-loop system including LQR balance controller and prediction is stable.');
    else
        disp('The closed-loop system including LQR balance controller and prediction is unstable.');
    end
end


%% Load the path
% Load all the predefined paths and select the chosen one
paths;

% Compute the cumulative distance and the curvature of the chosen path
cumulative_distance_curvature;


%% Run the Simulink simulation
try sim(simulink_file)
        
catch Error_Reason
    Error_Reason.message,
end


%% Draw plots
% Squeeze vectors with extra dimensions of length 1
states_measured.Data = squeeze(states_measured.Data);

draw_plots_2states;
% draw_plots_pathComparisonReport_20190820;
% draw_plots_controller_tuning;
% figure;plot(heading.Time,rad2deg(heading.Data));title('heading');


%% Compute path tracking performance indicators
% performance_indicators;