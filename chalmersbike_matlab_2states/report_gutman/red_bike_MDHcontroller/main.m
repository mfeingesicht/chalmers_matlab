% clear all
% close all
clc


%% Bike parameters
% Initial angles
initial_roll = deg2rad(0);
initial_rollrate = deg2rad(1);
initial_steering = deg2rad(0);

% Sampling time
Ts = 0.01;            % sampling time

% Total simulation time
sim_time = 20;

% Inertia front wheel
inertia_front = 0.2741;
inertia_front = 0;

% Model choice
uneven_mass = 0;
nonlinear_model = 0;

% Forward velocity
v = 14/3.6;                % velocity [m/s]

% Red bike physical parameters
r_wheel = 0.311;      % radius of the wheel
h = 0.2085 + r_wheel; % height of center of mass [m]
b = 1.095;            % length between wheel centers [m]
c = 0.06;             % length between front wheel contact point and the extention of the fork axis [m]
lambda = deg2rad(70); % angle of the fork axis [deg]
a = 0.4964;           % distance from rear wheel to frame's center of mass [m]
IMU_height = 0.45;    % IMU height [m]
g = 9.81;             % gravity [m/s^2]
m = 45;               % Bike mas [kg]
bike_params = [g h b a lambda c m];

% Complementary filter
complementary = 0.985;
roll_rate_coef = 0.9;

% IMU
load('imu_noise_ground.mat');
load('imu_noise_lifted.mat');
imu_gx_noise = [t noise_gx*pi/180];
imu_gx_offset = offset_gx*pi/180;


%% Steering motor
% Red bike
num_tf_steering = [253.4];
den_tf_steering = [1 245.9];
delay_steering = 0; % times steps

% % Black bike
% num_tf_steering = [50];
% den_tf_steering= [1 50];
% delay_steering = 4; % times steps

sys_tf_steering = tf(num_tf_steering,den_tf_steering);
sys_tf_steering_dis = c2d(sys_tf_steering,Ts);

P_steering_position = 30;
I_steering_position = 0;
D_steering_position = 0;
K_feedforward_steering = 0;


%% Balancing
P_balancing = 3.418;
I_balancing = 1.327;
D_balancing = 0.0646;

P_balancing = 0.5;
I_balancing = 0;
D_balancing = 1;


%% Path tracking
% Initial position (XY)
initial_x = 0;
initial_y = 0;

% Enable / Disable path tracking
path_tracking = 0;     % 1 = enable path tracking ;  0 = only self balancing, no path tracking

% Enable / Disable parts of the path tracking
lateral_control = 0;   % 1 = enable lateral position control ; 0 = disable
heading_control = 0;   % 1 = enable direction (heading) control ;  0 = disable

% PID gains for the path tracking
% Lateral control
P_lateral = -0.07;    % Lateral control P gain
I_lateral = 0;   % Lateral control I gain
D_lateral = 0.01;      % Lateral control D gain
% Direction control
P_heading = -0.1; % Direction control P gain
I_heading = -0.01; % Direction control I gain
D_heading = 0.01;   % Heading control D gain

% Select path and its characteristics
radius = 15; % radius
slope = 1/1000; % slope
path = 5;    % 1: Straight Path   2: Circle    3: _/-\_
             % 4: Sinusoidal path    5: _/-\_ integrated from heading
paths;
cumulative_distance_curvature;


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