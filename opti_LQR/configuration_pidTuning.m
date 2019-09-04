%% Simulink

% Simulink file
% simulink_file = 'bike_model.slx';
simulink_file = 'bike_model_pidTuning.slx';

% Simulation time
sim_time = 74;  % Simulation time [s] : Max: 89 sec (due to available noise data)

% Initial states
initial_states = [deg2rad(-2) deg2rad(0) deg2rad(0)];  % Initial states of the bike at t=0
                                                      % [roll angle in rad, steering angle in rad, roll rate in rad/s];
% % initial_states = [deg2rad(5) deg2rad(-2) deg2rad(0)];  % Initial states of the bike at t=0
                                                      % [roll angle in rad, steering angle in rad, roll rate in rad/s];

% Sampling Time
Ts = 0.04;     % Sampling time for the measurements and control loop [s]
Ts_IMU = 0.04; % Sampling time for the IMU [s]


%% Physical bike
% Velocity
input_velocity = 3;  % Forward velocity [m/s]

% Box position
box_center = 1;

% Real Bike Parameters
if box_center
    r_wheel = 0.311; % Radius of the wheel
    % Global system (frame+box as 1 volume)
    h_real = 0.2085 + r_wheel;           % height of center of mass [m]
    b_real = 1.095;            % length between wheel centers [m]
    c_real = 0.06;             % length between front wheel contact point and the 
                               % extention of the fork axis [m]
    lambda_real = deg2rad(70); % angle of the fork axis [deg]
    a_real = 0.4964;           % distance from rear wheel to frame's center of mass [m]
    IMU_height_real = 0.45;    % IMU height [m]
    g = 9.81;                  % gravity [m/s^2]

    % Bike model for complementary filter
    h_compFilter = h_real;              % height of center of mass [m]
    b_compFilter = b_real;              % length between wheel centers [m]
    c_compFilter = c_real;              % length between front wheel contact point and the 
                                        % extention of the fork axis
    lambda_compFilter = lambda_real;    % angle of the fork axis
    a_compFilter = a_real;              % distance from rear wheel to center of mass [m]
    IMU_height_model = 0.45;            % IMU height [m]    0.96 vs 0.90 vs 0.45

    % Differenciation frame and box (2 volumes)
    % Parameters of the bike
    m_frame_real = 35;         % mass of the frame [kg]
    h_frame_real = 0.51;       % height of frame's center of mass [m]
    a_frame_real = 0.38;       % distance from rear wheel to frame's center of mass [m] before 0.325
    c_frame_real = 0.04;       % length of the frame representative box along y axis [m]
    b_frame_real = 1;          % height of the frame representative box along z axis [m]
    l_frame_real = 1.8;        % length of the frame representative box along x axis [m]

    % Parameters of the box
    m_box_real = 12;           % mass of the box containing the electronics [kg]
    h_box_real = 0.49 + r_wheel;         % height of box's center of mass [m]  
    a_box_real = 0.67;         % distance from rear wheel to box's center of mass [m]
    c_box_real= 0.4;           % length of the representative box along y axis [m]
    b_box_real = 0.22;         % height of the representative box along z axis [m]
    l_box_real = 0.5;          % length of the representative box along x axis [m]
else
    % Global system (frame+box as 1 volume)
    h_real = 0.48+0.311;             % height of center of mass [m]
    % h_real = 0.48;             % height of center of mass [m]
    b_real = 1.095;            % length between wheel centers [m]
    c_real = 0.06;             % length between front wheel contact point and the 
                               % extention of the fork axis [m]
    lambda_real = deg2rad(70); % angle of the fork axis [deg]
    a_real = 0.34;             % b_real-0.77;
    IMU_height_real = 0.45;    % IMU height [m]
    g = 9.81;                  % gravity [m/s^2]

    % Differenciation frame and box (2 volumes)
    % Parameters of the bike
    m_frame_real = 35;         % mass of the frame [kg]
    h_frame_real = 0.51;       % height of frame's center of mass [m]
    a_frame_real = 0.38;       % distance from rear wheel to frame's center of mass [m] before 0.325
    c_frame_real = 0.04;       % length of the frame representative box along y axis [m]
    b_frame_real = 1;          % height of the frame representative box along z axis [m]
    l_frame_real = 1.8;        % length of the frame representative box along x axis [m]

    % Parameters of the box
    m_box_real = 12;           % mass of the box containing the electronics [kg]
    h_box_real = 0.95;         % height of box's center of mass [m]  
    a_box_real = 0.12;         % distance from rear wheel to box's center of mass [m]
    c_box_real= 0.4;           % length of the representative box along y axis [m]
    b_box_real = 0.22;         % height of the representative box along z axis [m]
    l_box_real = 0.5;          % length of the representative box along x axis [m]

    % Bike model for complementary filter
    h_compFilter = 0.48+0.311;                % height of center of mass [m]
    % h_compFilter = 0.48;                % height of center of mass [m]
    b_compFilter = 1.095;               % length between wheel centers [m]
    c_compFilter = 0.06;                % length between front wheel contact point and the 
                                        % extention of the fork axis
    lambda_compFilter = deg2rad(70);    % angle of the fork axis
    a_compFilter = b_compFilter - 0.77; % distance from rear wheel to center of mass [m]
    IMU_height_model = 0.45;            % IMU height [m]    0.96 vs 0.90 vs 0.45
end


% Store bike parameters in vectors
parameters_real = [g h_real b_real a_real lambda_real c_real];
parameters_f_b_real = [m_frame_real m_box_real h_frame_real h_box_real a_frame_real a_box_real c_frame_real c_box_real b_frame_real b_box_real l_frame_real l_box_real];
parameters_compFilter = [g h_compFilter b_compFilter a_compFilter lambda_compFilter c_compFilter];

% Steering Motor Limitations
Dead_Band = [0.055, -0.055, 0.005]; % for steering motor [Dead Band Limit(+,-), Relay]
                                    % [rad/sec]
max_steering_motor_acc = inf;       % maximum steering motor acceleration [rad/s^2]
min_steering_motor_acc = -inf;      % minimum steering motor acceleration [rad/s^2]
max_steering_motor_speed = 7.5;     % maximum steering motor velocity [rad/s]
min_steering_motor_speed = -7.5;    % minimum steering motor velocity [rad/s]
delay = 0.045;                      % delay between change in the reference and the output of the steering motor [s

% Angles limitations
max_roll= pi/2;                  % maximum roll angle permitted [rad] 
                                 % // otherwise simulation stops
max_handlebar=40*(pi/180);       % maximum handlebar angle [rad] before the saturation
min_roll= - max_roll;            % minimum roll angle permitted [rad] 
                                 % // otherwise simulation stops
min_handlebar=-max_handlebar;    % minumum handlebar angle [rad] before the saturation

% Look ahead / EXPERIMENTAL, NOT FINALIZED!
look_ahead = 0;      % 1 = enable look-ahead path control, 0=disable
                   
% Sensor noise
noise = 0;                         % 1 = enable noise on the sensors, 0 = disable
noise_hall = 0.0031;               % variance of hall sensor noise 
noise_encoder = 1e-7;              % variance of steering motor encoder noise
noise_gyro = 0.0022;               % variance of gyroscope noise
noise_acc = 2.4869e-05;            % variance of noise of roll angle estimation based on accelerometer
noise_pos_meas = 0.51;             % variance of position measurement system

% Hall sensor
dout = 0.7; % The diameter of the outer tyre frame
nrsensor = 5; % The number or magnets in hall sensor
sensordist = dout*pi/nrsensor; % The distances between different magnets
ToleranceVelocityEst = 0.5; % The tolerance for forward velocity filter


%% Model
% Oscillations due to the box
osc_roll = 0;       % 1 = enable oscillation on roll due to the box ; 0 = disable
osc_steer = 0;      % 1 = enable oscillation on steering due to the box ; 0 = disable
damp = 0.4;         % Damping of the box oscillation transfer function
fr_oscil = 1;       % Frequency [Hz] of the box oscillation transfer function
gain = 0.3;         % Gain of the box oscillation transfer function

% Model
fork_dynamics_lqr = 1;          % 1 = enable taking into account fork angle in the model ; 0 = disable
forward_motor_dynamics = 0;     % 1 = enable forward motor dynamics ;  0 = disable
steering_motor_limitations = 1; % 1 = enable limitations (deadband, delay) ; 0 = disable
path_tracking = 1;              % 1 = enable path tracking ;  0 = only self balancing, no path tracking
path_tracking_indicator = 1;    % 1 = enable the computation of the performance indicators for path tracking ; 0 = disable

plant_model = 1;  % Choice of the model used to represent the bike
                  % 1 = nonlinear plant ; 2 = linear plant
                  % 3 = linear plant with neglected fork angle (the simplest plant)

filter_model = 1; % Choice of the model used for the filter estimating the lateral acceleration due to roll
                  % 1 = use measurements for compensation
                  % 2 = use model to estimate the necessary values for compensation 
                  % (with IC, but not with the recent measurement)
                  % 3 = use model to estimate the necessary values for compensation 
                  % (with IC, and then iterate autonomously)


%% LQR control
% LQR weights
% Q = diag([50,50,10]); % cost of states in LQR controller
% R = 42; 

Q = diag([20,100,10]); % cost of states in LQR controller
R = 2000; 

% Output matrices
C = eye(3);      % we assume that we measure all states ; does not depent on velocity
D = zeros(3,1);  % we assume that inputs don't affect outputs directly ; does not depent on velocity


%% Complementary filter
% Filter coefficients
complementary = 0.985;  % complementary filter coefficient (0.5 < c < 1) 
roll_rate_coef = 0.9;   % coefficient of the low pass filter at the gyro output
                  

%% Path tracking
% Select path and its characteristics
radius = 15; % radius for paths 2, 4, 10, 11, 12, 14, 15 [m]
slope = 0.3; % slope for paths 3, 5, 6
path = 1;    % 1: Straight Path --   2: Circle       3:_/-       4: ___O___
             % 5: Straight Path with Oscillations 1 /
             % 6: Straight Path with Oscillations 2 /
             % 7: Straight Path /                    8: Sinusoidal Path
             % 9: Increasing Oscillating Sinusoidal Path
             %10: 45? curve (set radius)
             %11: 90? curve (set radius)
             %12: Straight, turn along curve, straight back (set radius)
             %14: overtaking
             %15: _/- with 45? angles

% Enable / Disable parts of the path tracking
lat_control = 1;      % 1 = enable lateral control ; 0 = disable
direct_control = 1;   % 1 = enable direction control ;  0 = disable
steer_control = 1;    % 1 = enable steering angle (delta) control ;  0 = disable

% PID settings for the path tracking
% Lateral control
P_lat = 0.1;    % Lateral control P gain
I_lat = 0.01;   % Lateral control I gain
D_lat = 0;      % Lateral control D gain
% Direction control
P_direct = 0.8; % Direction control P gain
I_direct = 0.4; % Direction control I gain
D_direct = 0;   % Direction control D gain
% Steering control
P_steer = 1;    % Steering control P gain
I_steer = 0.1;  % Steering control I gain
D_steer = 0;    % Steering control D gain

% PID settings for the path tracking
% Nonzero initial conditions : 2° initial roll angle
% Obtained by tuning the PID blocks in bike_model_pidTuning
% Lateral control
P_lat = 0.0465435246389258;    % Lateral control P gain
I_lat = 0.000115580509653549;   % Lateral control I gain
D_lat = 0;      % Lateral control D gain
% Direction control
P_direct = 0.126067479140802; % Direction control P gain
I_direct = 0.000739995118675849; % Direction control I gain
D_direct = 0;   % Direction control D gain
% Steering control
P_steer = 0.219020351848633;    % Steering control P gain
I_steer = 0.4510120195392;  % Steering control I gain
D_steer = 0;    % Steering control D gain

% PID settings for the path tracking
% Zero initial conditions
% Obtained by tuning the PID blocks in bike_model_pidTuning
% Lateral control
P_lat = 0.0465435246389258;    % Lateral control P gain
I_lat = 0.000115580509653549;   % Lateral control I gain
D_lat = 0;      % Lateral control D gain
% Direction control
P_direct = 0.0712859744206787; % Direction control P gain
I_direct = 0.000227125326188444; % Direction control I gain
D_direct = 0;   % Direction control D gain
% Steering control
P_steer = 0.219020351848633;    % Steering control P gain
I_steer = 0.4510120195392;  % Steering control I gain
D_steer = 0;    % Steering control D gain

% Kalman filter - Position estimation
% State matrices
A_kalman = [1 0 Ts 0; 0 1 0 Ts; 0 0 1 0; 0 0 0 1];
B_kalman = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
C_kalman = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
D_kalman = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];

% Noise matrices
Q_kalman = [0.0001 0 0 0; 0 0.0001 0 0; 0 0 0.1 0; 0 0 0 0.1];  % Process noise covariance matrix
R_kalman = [noise_pos_meas 0 0 0; 0 noise_pos_meas 0 0; 0 0 noise_hall 0; 0 0 0 noise_hall];  % Measurement noise covariance matrix


%% Plot settings
% LQR plots
plots = [0,0,0];   % 1 = enable LQR plots [eigenvalue plot, K plot, path plot]

% Bike animation
Bike_animation = 0;              %Set to 1 to see bike animation (path and 3D simulation
scaleP = 4;                      %Animation Play Speed, Number of time steps at each loop 

% Bike plots
Bike_Path_vs_Reference_Path = 1; %Set to 1 to see bike path vs reference path
Bike_path_vs_ref_axis_equal = 1; %Set to 1 to see bike path vs reference path in equal axis plot
Kalman_vs_Real = 0;              %Set to 1 to see the error of the Kalman position estimation compared to real position
Bike_path_estimations = 1;       %Set to 1 to see Kalman/pos meas/vel meas integration
Estimations_errors = 1;
Bike_Path_Error_Inscpection = 1; %Set to 1 to see bike path errors 
States_Plot = 0;                 %Set to 1 to see True/Measured states plot
States_True = 1;                 %Set to 1 to see True states plot