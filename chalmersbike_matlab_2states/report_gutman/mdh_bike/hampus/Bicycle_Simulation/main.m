% clear all
% close all
clc


%% Bike parameters
% Initial angles
% initial_roll = deg2rad(2);
initial_roll = deg2rad(0);
initial_rollrate = deg2rad(0);
initial_steering = deg2rad(0);

% Sampling time
Ts = 0.01;            % sampling time

% Total simulation time
sim_time = 50;

% Inertia front wheel
inertia_front = 0.2741;
inertia_front = 0;

% Model choice
uneven_mass = 0;
nonlinear_model = 0;

% Forward velocity
v = 14/3.6;                % velocity [m/s]

% Bike physical parameters
h = 0.562;            % height of center of mass [m]
b = 1.115;            % length between wheel centers [m]
a = 0.505;            % distance from rear wheel to frame's center of mass [m]
IMU_height = 0.2;    % IMU height [m]
g = 9.82;             % gravity [m/s^2]
m = 25;               % Bike mas [kg]
lambda = deg2rad(90-24); % angle of the fork axis [deg]
bike_params = [g h b a lambda m]; % Store bike parameters in a vector

% Complementary filter
complementary = 0.985;
roll_rate_coef = 0.9;


%% Balancing
P_balancing = 3.418;
I_balancing = 1.327;
D_balancing = 0.0646;


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