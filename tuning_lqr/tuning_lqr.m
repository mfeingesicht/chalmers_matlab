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
configuration_lqrTuning;


%% Create list of weight matrices to test
%     [ Q11 Q12 Q13 ]
% Q = [ Q12 Q22 Q23 ]=> [Q11 Q22 Q33 Q12 Q13 Q23]
%     [ Q13 Q23 Q33 ]
% 
% 
%                [ Q11_1 Q22_1 Q33_1 Q12_1 Q13_1 Q23_1 ]
% Q_comparison = [ Q11_2 Q22_2 Q33_2 Q12_2 Q13_2 Q23_2 ]
%                [ Q11_3 Q22_3 Q33_3 Q12_3 Q13_3 Q23_3 ]
% 
%                [ R_1 ]
% R_comparison = [ R_2 ]
%                [ R_3 ]
% 

% To use later as 'fast' LQR controllers
% Q = [80 80 40 40 0 40]
% Q = [160 80 40 40 0 40]

% To use later as 'slow' LQR controllers
% Q = []


% Q13 seems useless
% Q_comparison = [80 80 40 0 0 0 ; 80 80 40 50 0 0 ; 80 80 40 0 0 50 ; 800 400 200 0 0 0];
Q_comparison = [160 80 40 40 0 40 ; 160 160 160 160 0 40];

% R_comparison = [1 ; 1 ; 1 ; 1];
R_comparison = ones(size(Q_comparison,1),1);


%% Create vectors to store the simulation results
time = cell(size(Q_comparison,1),1);
roll_comparison = cell(size(Q_comparison,1),1);
steering_comparison = cell(size(Q_comparison,1),1);
rollrate_comparison = cell(size(Q_comparison,1),1);


%% Run simulation for each set of weights
for index=1:size(Q_comparison,1)
    R = R_comparison(index);
    Q = diag(Q_comparison(index,1:3)) + squareform(Q_comparison(index,4:end));
    
    % Check if matrices are positive definite
    eigQ = eig(Q);
    
    if ( (R>0) && (all(eigQ>=0)) )
        %% LQR controller
        lqr_design;


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
        
        time{index} = states_true.time;
        roll_comparison{index} = rad2deg(states_true.data(:,1));
        steering_comparison{index} = rad2deg(states_true.data(:,2));
        rollrate_comparison{index} = rad2deg(states_true.data(:,3));
    else
        disp(['Weight matrix Q number ' num2str(index) ' is not positive semi-definite.']);
        time{index} = [];
        roll_comparison{index} = [];
        steering_comparison{index} = [];
        rollrate_comparison{index} = [];
    end
end


%% Plot all simulations together
draw_plots_lqr_testing;