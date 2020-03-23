
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


% Data_exp = readtable('C:\Users\maximef\Google Drive\chalmersbike\New Folder (2)\BikeData-20190910-163107.csv');
% initial_states = [Data_exp.Phi(76) Data_exp.Delta(76) Data_exp.PhiDot(76)];


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

% To use as 'fast' LQR controllers
% Q_comparison = [80 80 40 40 0 40];
% Q_comparison = [160 80 40 40 0 40];

% To use as 'medium' LQR controllers
% Q_comparison = [10 10 10 10 0 0 ; 1 1 10 1 0 0];

% To use as 'slow' LQR controllers
% Q_comparison = [5 5 5 0 0 5];
% R = 0.1;

% Q13 seems useless
% Q_comparison = [80 80 40 0 0 0 ; 80 80 40 50 0 0 ; 80 80 40 0 0 50 ; 800 400 200 0 0 0];
% Q_comparison = [160 80 40 40 0 40 ; 160 160 160 160 0 40];

% R_comparison = ones(size(Q_comparison,1),1);
% R_comparison = 1*ones(size(Q_comparison,1),1);
% R_comparison = [1 10 0.1];



% Q_comparison = [10 10 10 10 0 0 ; 10 10 10 0 0 10 ; 5 5 5 5 0 0 ; 7.5 7.5 7.5 7.5 0 0];
% Q_comparison = [10 10 10 10 0 0 ; 10 10 10 0 0 10 ; 100 100 100 100 0 0 ; 5 5 5 5 0 0 ; 7.5 7.5 7.5 7.5 0 0 ; 5 5 5 0 0 5 ; 7.5 7.5 7.5 0 0 7.5];
% Q_comparison = [100 100 100 0 0 0 ; 100 100 100 100 0 0 ; 100 100 100 0 0 100];
% Q_comparison = [100 100 100 100 0 0 ; 100 100 100 0 0 100 ; 10 10 10 0 0 0 ; 10 10 10 10 0 0 ; 10 10 10 0 0 10];
% Q_comparison = [10 10 10 10 0 0 ; 10 10 10 0 0 10 ; 100 100 100 100 0 0 ; 100 100 100 0 0 100 ; 1 1 1 1 0 0 ; 1 1 1 0 0 1];

% Q_comparison = [1000 10 1 0 0 1 ; 100 10 1 0 0 1 ; 10 10 1 0 0 1 ; 1 10 1 0 0 1 ; 10 1 1 0 0 1 ; 100 100 100 0 0 100 ; 100 100 100 100 0 0];
% Q_comparison = [200 200 200 0 0 200 ; 100 100 100 0 0 100 ; 100 100 100 100 0 0];
% R_comparison = 1*ones(size(Q_comparison,1),1);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% VALUES TO KEEP ('fast ; 'medium' ; 'slow')
% Q_comparison = [10 1 1 0 0 1 ; 100 100 100 0 0 100 ; 100 100 100 100 0 0];
% Q_comparison = [2000 2000 100 0 0 200 ; 100 100 100 0 0 100 ; 100 100 100 100 0 0];
% Q_comparison = [200 200 200 0 0 5 ; 100 100 100 0 0 100 ; 100 100 100 100 0 0];
Q_comparison = [10 10 1 0 0 1 ; 100 100 100 0 0 100 ; 100 100 100 100 0 0];
R_comparison = 1*ones(size(Q_comparison,1),1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Q_comparison = [1 1 1 1 0 0 ; 0.1 0.1 0.1 0.1 0 0];
Q_comparison = [0.1 0.1 0.1 0.1 0 0 ; 0.1 0.01 0.1 0.01 0 0 ; 100 20 20 0.01 0 0];
R_comparison = 1*ones(size(Q_comparison,1),1);


Q_comparison = [100 100 100 0 0 100 ; 100 100 100 100 0 0 ; 1 0.01 1 0 0 0 ; 1 1 1 0 0 0];
R_comparison = 1*ones(size(Q_comparison,1),1);


Q_comparison = [1 1 1 1 0 0 ; 1 1 1 0 0 1 ; 1 1 1 0 0 0 ; 1 0.01 1 0 0 0 ; 1 1 0.01 0 0 0 ; 1 0.01 0.01 0 0 0 ; 0.01 0.01 0.01 0 0 0];
Q_comparison = [1 1 1 1 0 0 ; 1 1 1 0 0 1 ; 1 1 1 0 0 0 ; 1 100 1 0 0 0 ; 1 1 100 0 0 0 ; 1 100 100 0 0 0 ; 100 100 100 0 0 0];
Q_comparison = [100 10 1 1 0 0];
R_comparison = 1*ones(size(Q_comparison,1),1);



%% Create vectors to store the simulation results
time = cell(size(Q_comparison,1),1);
roll_comparison = cell(size(Q_comparison,1),1);
steering_comparison = cell(size(Q_comparison,1),1);
rollrate_comparison = cell(size(Q_comparison,1),1);
steeringrate_comparison = cell(size(Q_comparison,1),1);

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
        steeringrate_comparison{index} = rad2deg(steeringrate.data);
    else
        disp(['Weight matrix Q number ' num2str(index) ' is not positive semi-definite.']);
        time{index} = [];
        roll_comparison{index} = [];
        steering_comparison{index} = [];
        rollrate_comparison{index} = [];
        steeringrate_comparison{index} = [];
    end
end


%% Plot all simulations together
draw_plots_lqr_testing;
figure;bode(sys_dis)
sys_CL_dis = ss(sys_dis.A-sys_dis.B*[lqr_gains(1) lqr_gains(2) 0],sys_dis.B*[lqr_gains(1) lqr_gains(2) 0],sys_dis.C,0,Ts);
sys_CL_dis = ss(sys_dis.A-sys_dis.B*[lqr_gains(1) lqr_gains(2) lqr_gains(3)],sys_dis.B*[lqr_gains(1) lqr_gains(2) lqr_gains(3)],sys_dis.C,0,Ts);
figure;bode(sys_CL_dis)