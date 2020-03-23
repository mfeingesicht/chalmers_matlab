clear all
close all
clc


%% Bike parameters
sim_time = 20;

% Bike choice
bike = 'red';
% bike = 'black';

% Initial angles
initial_roll = deg2rad(2);
initial_rollrate = deg2rad(2);
initial_steering = deg2rad(0);

% Sampling time
Ts = 0.01;            % sampling time

% Inertia front wheel
% inertia_front = 0.2741;
inertia_front = 0.245;
% inertia_front = 0;

% Model choice
uneven_mass = 0;
nonlinear_model = 0;

% Forward velocity
% v = 3;                % velocity [m/s]
v = 4;                % velocity [m/s]

% Bike physical parameters
if strcmp(bike,'red')
    % Red bike
    r_wheel = 0.311;      % radius of the wheel
    h = 0.2085 + r_wheel; % height of center of mass [m]
    b = 1.095;            % length between wheel centers [m]
    c = 0.06;             % length between front wheel contact point and the extention of the fork axis [m]
    lambda = deg2rad(70); % angle of the fork axis [deg]
    a = 0.4964;           % distance from rear wheel to frame's center of mass [m]
    IMU_height = 0.45;    % IMU height [m]
    g = 9.81;             % gravity [m/s^2]
    m = 45;               % Bike mas [kg]
elseif strcmp(bike,'black')
    % Black bike
    h = 0.534;            % height of center of mass [m]
    b = 1.15;             % length between wheel centers [m]
    a = 0.6687;           % distance from rear wheel to frame's center of mass [m]
    IMU_height = 0.215;   % IMU height [m]
    g = 9.81;             % gravity [m/s^2]
    m = 31.3;             % Bike mass [kg]
    lambda = deg2rad(90-24); % angle of the fork axis [deg]  !!! TO BE MEASURED
    bike_params = [g h b a lambda m]; % Store bike parameters in a vector
else
    % Choose red bike as default
    r_wheel = 0.311;      % radius of the wheel
    h = 0.2085 + r_wheel; % height of center of mass [m]
    b = 1.095;            % length between wheel centers [m]
    c = 0.06;             % length between front wheel contact point and the extention of the fork axis [m]
    lambda = deg2rad(70); % angle of the fork axis [deg]
    a = 0.4964;           % distance from rear wheel to frame's center of mass [m]
    IMU_height = 0.45;    % IMU height [m]
    g = 9.81;             % gravity [m/s^2]
    m = 45;               % Bike mas [kg]
end

% Complementary filter
complementary = 0.985;
roll_rate_coef = 0.9;


%% Bike transfer function
num = [(inertia_front*b)/(m*h) a*v v^2];
den = [b*h 0 -b*g];
sys_tf = tf(num,den);


%% Bike state-space matrices
% Continuous time
[A,B,C,D] = tf2ss(num,den);
sys = ss(A,B,C,D);
sys_fullstates = ss(sys.A,sys.B,eye(size(sys.A)),0);

% Discrete time
sys_dis = c2d(sys,Ts);
A_dis = sys_dis.A;
B_dis = sys_dis.B;
C_dis = sys_dis.C;
D_dis = sys_dis.D;


%% PID Balancing Controller
P_balancing = 1;
I_balancing = 0;
D_balancing = 0.05;


%% Steering motor
num_tf_steering = [253.4];
den_tf_steering = [1 245.9 0];
sys_tf_steering = tf(num_tf_steering,den_tf_steering);

sys_tf_steering_dis = c2d(sys_tf_steering,Ts);

P_steering_position = 30;
I_steering_position = 30;
D_steering_position = 0;
K_feedforward_steering = 0;


%% Butterworth LP filter
[bb,ba]=butter(2,1/(100/2));
% [bb,ba]=butter(1,6/(100/2));


%% Path tracking
% Initial position (XY)
initial_x = 0;
initial_y = 0;

% Enable / Disable path tracking
path_tracking = 1;     % 1 = enable path tracking ;  0 = only self balancing, no path tracking

% Enable / Disable parts of the path tracking
lateral_control = 0;   % 1 = enable lateral position control ; 0 = disable
heading_control = 1;   % 1 = enable direction (heading) control ;  0 = disable

% PID gains for the path tracking
% Lateral control
P_lateral = -0.2;    % Lateral control P gain
I_lateral = 0;   % Lateral control I gain
D_lateral = -0.1;      % Lateral control D gain
% Direction control
P_heading = -0.4; % Direction control P gain
I_heading = 0; % Direction control I gain
D_heading = 0.05;   % Heading control D gain

% P_heading = -0.4; D_heading = 0.05;
% P_heading = -0.1; D_heading = 0.001;
P_heading = -0.1; D_heading = 0.01;
% P_heading = -0.1; D_heading = 0.05;
% P_heading = -0.1; D_heading = 0.1;

P_heading = -0.4; D_heading = 0.05;
% P_heading = -0.1; D_heading = 0.01;
% P_heading = -0.01; D_heading = 0.0001;
P_heading = -0.05; D_heading = 0.01;
P_heading = -0.05; D_heading = 0.05;
P_heading = -0.05; D_heading = 0.1;

P_heading = -0.2; D_heading = 0.05;
% P_heading = -0.2; D_heading = 0.1;

% Select path and its characteristics
radius = 15; % radius
slope = 1/1000; % slope
path = 5;    % 1: Straight Path   2: Circle    3: _/-\_
             % 4: Sinusoidal path    5: _/-\_ integrated from heading
paths;
cumulative_distance_curvature;