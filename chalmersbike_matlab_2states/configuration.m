%% Simulink

% Simulink file
% simulink_file = 'bike_model_2states_smithSteering.slx';
simulink_file = 'bike_model_2states_smithFullBike.slx';

% Simulation time
sim_time = 74;  % Simulation time [s]
sim_time = 1000;  % Simulation time [s]
sim_time = 40;  % Simulation time [s]

% Initial angles
% From data :
% -4 < phi < 4
% -6 < delta < 6
% -2 < phidot < 2
initial_states = [deg2rad(-2) deg2rad(0) deg2rad(0)];  % Initial angles of the bike at t=0
                                                       % [roll angle in rad, steering angle in rad, roll rate in rad/s];
initial_states = [deg2rad(0) deg2rad(0) deg2rad(0)];
initial_states = [deg2rad(-2) deg2rad(0) deg2rad(0)];

% Sampling Time
Ts = 0.01;     % Sampling time for the measurements and control loop [s]
Ts_IMU = 0.01; % Sampling time for the IMU [s]

% Choose LQR controller type
% lqr_type = 'fast';
lqr_type = 'medium';
% lqr_type = 'slow';

% Smith predictor
smith_steering = 0;
K_Steering = 1;

smith_fullbike = 1;
K_fullbike = 1;

% Steering angle low-pass filter
% order_filter_delta = 1;
order_filter_delta = 2;
f_filter_delta = 1; %Hz
f_filter_delta = 8; %Hz

% Steering filter
[b_steering_filter,a_steering_filter]=butter(1,1/((1/Ts)/2),'low');
% [b_steering_filter,a_steering_filter]=butter(2,3/((1/Ts)/2),'low');
% % [b_steering_bandstop,a_steering_bandstop]=butter(1,[1 3]/((1/Ts)/2),'stop');
% [b_steering_filter,a_steering_filter]=butter(1,[1.5 2.5]/((1/Ts)/2),'stop');
figure;freqz(b_steering_filter,a_steering_filter,2048,(1/Ts));

% Roll rate low-pass filter
% order_filter_phidot = 1;
order_filter_phidot = 2;

f_filter_phidot = 3; %Hz
% f_filter_phidot = 6; %Hz
f_filter_phidot = 20; %Hz

[num_filter_phidot,den_filter_phidot] = butter(order_filter_phidot,f_filter_phidot/(1/(2*Ts)),'s');


%% Physical bike
% Velocity
input_velocity = 3;  % Forward velocity [m/s]
input_velocity = 4;  % Forward velocity [m/s]
% input_velocity = 1;  % Forward velocity [m/s]

% Box position
box_center = 1;

% Mass distribution
uneven_mass = 0;
d_uneven_mass = 0.07;

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
    m_box_real = 9.3;           % mass of the box containing the electronics [kg]
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
    m_box_real = 9.3;           % mass of the box containing the electronics [kg]
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


%% State-space matrices
% A_bike = [0 12*g/h_real ; 1 0];
% B_bike = [1 ; 0];
% C1_bike = 12*a_real/(b_real*h_real); % To be multiplied by v in Simulink
% C2_bike = 12/(b_real*h_real); % To be multiplied by v^2 in Simulink
% D_bike = 0;

A_bike = [0 g/h_real ; 1 0];
B_bike = [1 ; 0];
C1_bike = a_real/(b_real*h_real); % To be multiplied by v in Simulink
C2_bike = 1/(b_real*h_real); % To be multiplied by v^2 in Simulink
D_bike = 0;

[n,m] = size(A_bike);

sys_bike = ss(A_bike,B_bike,eye(n),0);
sys_bike_dis = c2d(sys_bike,Ts);
A_bike_dis = sys_bike_dis.A;
B_bike_dis = sys_bike_dis.B;
C1_bike_dis = C1_bike;
C2_bike_dis = C2_bike;
D_bike_dis = D_bike;

C_ini = [C1_bike*input_velocity C2_bike*input_velocity^2];

% Discrete transfer function
num1_bike_dis = [2*a_real*Ts , 0 , -2*a_real*Ts];% To be multiplied by v in Simulink
num2_bike_dis = [Ts^2 , 2*Ts^2 , Ts^2];% To be multiplied by v^2 in Simulink
den_bike_dis = [4*b_real*h_real-b_real*g*Ts^2 , -8*b_real*h_real-2*b_real*g*Ts^2 , 4*b_real*h_real-b_real*g*Ts^2];


%% Steering Motor
Dead_Band = [0.055, -0.055, 0.005]; % for steering motor [Dead Band Limit(+,-), Relay]
                                    % [rad/sec]
max_steering_motor_acc = inf;       % maximum steering motor acceleration [rad/s^2]
min_steering_motor_acc = -inf;      % minimum steering motor acceleration [rad/s^2]
max_steering_motor_speed = 0.9;%2*pi;     % maximum steering motor velocity [rad/s]
min_steering_motor_speed = -0.9;%-2*pi;    % minimum steering motor velocity [rad/s]
delay = 0.045;                      % delay between change in the reference and the output of the steering motor [s

% Transfer function : Steering rate reference to steering rate measured
% 1 pole
num_tf_steering = [18.91];
% den_tf_steering = [1 17.9];
den_tf_steering = [1 17.9 0];

% delay_tf_steering = 0.00; % s
% delay_tf_steering = 0.04; % s
delay_tf_steering = 0.08; % s
% delay_tf_steering = 0.12; % s
% delay_tf_steering = 0.20; % s

tf_steering = tf(num_tf_steering,den_tf_steering);
% tf_steering = tf(num_tf_steering,den_tf_steering,'IODelay',delay_tf_steering);

% Discrete transfer function
tf_steering_dis = c2d(tf_steering,Ts);
num_tf_steering_dis = tf_steering_dis.num{1}(find(tf_steering_dis.num{1} ~= 0, 1, 'first'):end);
den_tf_steering_dis = tf_steering_dis.den{1}(find(tf_steering_dis.den{1} ~= 0, 1, 'first'):end);

% Identification as discrete TF
% 1 pole
delay_tf_steering = 0.08;
num_tf_steering_dis = [0.252 0];
den_tf_steering_dis = [1 -0.7627];
% Add integrator
num_tf_steering_dis = [num_tf_steering_dis 0];
den_tf_steering_dis = [den_tf_steering_dis 0] - [0 den_tf_steering_dis];
tf_steering_dis = tf(num_tf_steering_dis,den_tf_steering_dis,Ts);

% 2 poles
delay_tf_steering = 0.04;
num_tf_steering_dis = [0.2269 0];
den_tf_steering_dis = [1.0000 -1.2628 0.4787];
% Add integrator
num_tf_steering_dis = [num_tf_steering_dis 0];
den_tf_steering_dis = [den_tf_steering_dis 0] - [0 den_tf_steering_dis];
tf_steering_dis = tf(num_tf_steering_dis,den_tf_steering_dis,Ts);


delay_tf_steering = 0;
num_tf_steering = [253.4];
den_tf_steering = [1 245.9 0];
% Add integrator
num_tf_steering_dis = [num_tf_steering_dis 0];
den_tf_steering_dis = [den_tf_steering_dis 0] - [0 den_tf_steering_dis];
tf_steering_dis = tf(num_tf_steering_dis,den_tf_steering_dis,Ts);


% Steering angle reference to steering rate PID controller
P_steering_position = 10;
I_steering_position = 1;
D_steering_position = 0.0;
% K_feedforward_steering = 1.0;
% K_feedforward_steering = 0.95;
K_feedforward_steering = 0;
% K_feedforward_steering = 245.9/253.4;
% K_feedforward_steering = 245.9/253.4/3;

% Matrices for the prediction of d_delay steps ahead
d_delay = delay_tf_steering/Ts;
% d_delay = 1+(delay_tf_steering/Ts);
vec_powers_A = [];
vec_powers_AB = [];
for i=1:d_delay
    vec_powers_A = [vec_powers_A , A_bike_dis^(d_delay-i)];
    vec_powers_AB = [vec_powers_AB , A_bike_dis^(d_delay-i)*B_bike_dis];
end
A_bike_dis_prediction = A_bike_dis^d_delay;
B_bike_dis_prediction = vec_powers_AB;


%% Angles limitations
max_roll= pi/2;                  % maximum roll angle permitted [rad] 
                                 % // otherwise simulation stops
min_roll= - max_roll;            % minimum roll angle permitted [rad] 
                                 % // otherwise simulation stops
max_handlebar=40*(pi/180);       % maximum handlebar angle [rad] before the saturation
min_handlebar=-max_handlebar;    % minumum handlebar angle [rad] before the saturation


%% Look ahead / EXPERIMENTAL, NOT FINALIZED!
look_ahead = 0;      % 1 = enable look-ahead path control, 0=disable
        

%% Sensor noise
noise = 0;                      % 1 = enable noise on the sensors, 0 = disable
noise_hall = 0.0031;            % variance of hall sensor noise 
noise_encoder = 1e-7;           % variance of steering motor encoder noise
noise_gyro = 0.005;             % variance of gyroscope noise
noise_acc = 2.4869e-05;         % variance of noise of roll angle estimation based on accelerometer
noise_pos_meas = 0.51;          % variance of position measurement system
noise_pos_meas = 0;             % variance of position measurement system


%% Sensor offset (step)
sensor_offset = 0;              % 1 = enable noise on the sensors, 0 = disable

offset_hall = 0.0;           % amplitude of hall sensor offset step
time_offset_hall = 7;      % time of hall sensor offset step

offset_encoder = deg2rad(1);        % amplitude of steering motor encoder offset step
time_offset_encoder = 10;   % time of steering motor encoder offset step

offset_gyro = deg2rad(0);           % amplitude of gyroscope offset step
time_offset_gyro = 7;      % time of gyroscope offset step

offset_acc = 0.0*g;            % amplitude of accelerometer offset step
time_offset_acc = 7;       % time of accelerometer offset step

offset_pos_x = 0;          % amplitude of x-axis position measurement offset  step
time_offset_pos_x = 7;     % time of x-axis position measurement offset step

offset_pos_y = 0;          % amplitude of y-axis position measurement offset  step
time_offset_pos_y  = 7;    % time of y-axis position measurement offset step

% Forward speed offset
forward_speed_offset = 0; % m/s

% Steering angle correction
steering_angle_correction = deg2rad(0); % rad
% steering_angle_correction = deg2rad(5.9); % rad


%% Hall sensor
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
% fork_dynamics_lqr = 1;          % 1 = enable taking into account fork angle in the model ; 0 = disable
% forward_motor_dynamics = 0;     % 1 = enable forward motor dynamics ;  0 = disable
% steering_motor_limitations = 1; % 1 = enable limitations (deadband, delay) ; 0 = disable
fork_dynamics_lqr = 0;          % 1 = enable taking into account fork angle in the model ; 0 = disable
forward_motor_dynamics = 0;     % 1 = enable forward motor dynamics ;  0 = disable
steering_motor_limitations = 0; % 1 = enable limitations (deadband, delay) ; 0 = disable
path_tracking = 0;              % 1 = enable path tracking ;  0 = only self balancing, no path tracking
path_tracking_indicator = 1;    % 1 = enable the computation of the performance indicators for path tracking ; 0 = disable

plant_model = 2;  % Choice of the model used to represent the bike
                  % 1 = nonlinear plant
                  % 2 = linear plant with box and bike as 1 volume
                  % 3 = linear plant with box and bike as 2 volumes
                  %%%%%% 4 = linear plant with neglected fork angle (the simplest plant)

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

% Q = diag([2000,10,100]); % cost of states in LQR controller
% R = 2000; 

% % GA optimization LQR
% Q = [15.9549   5.1214   -5.7298
%      5.1214   27.6373   21.6896
%     -5.7298   21.6896   27.3988];
% R = 0.2336;
% % R = 6;

% % % % % % Output matrices
% % % % % C = eye(3);      % we assume that we measure all states ; does not depent on velocity
% % % % % D = zeros(3,1);  % we assume that inputs don't affect outputs directly ; does not depent on velocity


% switch lqr_type
%     case 'fast'
%         % 'fast' LQR
%         Q = [10 0 0 ; 0 10 1 ; 0 1 1];
%         R = 1;
%     case 'medium'
%         % 'medium' LQR
%         Q = [100 0 0 ; 0 100 100 ; 0 100 100];
%         R = 1;
%     case 'slow'
%         % 'slow' LQR
%         Q = [100 100 0 ; 100 100 0 ; 0 0 100];
%         R = 1;
% end


% Q and R weight matrices in phi and phi_dot
% The matrices are transformed to states x1 and x2 in the lqr_design script
Q_phi = [10 -9 ; -9 10];
R_phi = 1;


%% Complementary filter
% Filter coefficients
complementary = 0.985;  % complementary filter coefficient (0.5 < c < 1) 
roll_rate_coef = 0.9;   % coefficient of the low pass filter at the gyro output

% complementary = 0.5;  % complementary filter coefficient (0.5 < c < 1) 



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
lat_control = 0;      % 1 = enable lateral control ; 0 = disable
direct_control = 1;   % 1 = enable direction control ;  0 = disable
steer_control = 0;    % 1 = enable steering angle (delta) control ;  0 = disable

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


switch lqr_type
    case 'fast'
        % 'fast' LQR
        K_dir = [0.2 0 0];
        K_pos = [0.4 0 0];
    case 'medium'
        % 'medium' LQR
        K_dir = [0.4 0 0];
        K_pos = [0.185 0 0];
    case 'slow'
        % 'slow' LQR
        K_dir = [0.2 0 0];
        K_pos = [0.02 0 0];
end


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
Bike_path_estimations = 0;       %Set to 1 to see Kalman/pos meas/vel meas integration
% Estimations_errors = 1;
Estimations_errors = 0;
Bike_Path_Error_Inscpection = 1; %Set to 1 to see bike path errors 
States_Plot = 0;                 %Set to 1 to see True/Measured states plot
States_True = 1;                 %Set to 1 to see True states plot
Roll_RollRate_Plot = 1;          %Set to 1 to see Roll and Roll Rate plot