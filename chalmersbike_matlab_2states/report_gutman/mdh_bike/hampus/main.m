% clear all
% close all
clc


%% Bike parameters
% Plots
plots = 0 ;

% Initial angles
initial_roll = deg2rad(2);
initial_rollrate = deg2rad(2);
initial_steering = deg2rad(0);

% Sampling time
Ts = 0.01;            % sampling time

% Inertia front wheel
inertia_front = 0.2741;
inertia_front = 0;

% Model choice
uneven_mass = 0;
nonlinear_model = 0;

% Forward velocity
% v = 3;                % velocity [m/s]
v = 4;                % velocity [m/s]
% v = 5;                % velocity [m/s]

% Bike physical parameters
h = 0.562;            % height of center of mass [m]
b = 1.115;            % length between wheel centers [m]
a = 0.505;            % distance from rear wheel to frame's center of mass [m]
IMU_height = 0.45;    % IMU height [m]
g = 9.81;             % gravity [m/s^2]
m = 45;               % Bike mas [kg]
lambda = deg2rad(90-24); % angle of the fork axis [deg]
bike_params = [g h b a lambda m]; % Store bike parameters in a vector


% Complementary filter
complementary = 0.985;
roll_rate_coef = 0.9;


%% Balancing
% P_balancing = 2.5117;
% I_balancing = 1.5431;
% D_balancing = 0.0750;

P_balancing = 8.879;
I_balancing = 2.738;
D_balancing = 0.03562;


%% Path tracking
% Initial position (XY)
initial_x = 0;
initial_y = 0;

% Enable / Disable path tracking
path_tracking = 0;              % 1 = enable path tracking ;  0 = only self balancing, no path tracking

% Enable / Disable parts of the path tracking
lat_control = 0;      % 1 = enable lateral position control ; 0 = disable
direct_control = 1;   % 1 = enable direction (heading) control ;  0 = disable

% PID gains for the path tracking
% Lateral control
P_lat = 1;    % Lateral control P gain
I_lat = 0.1;   % Lateral control I gain
D_lat = 0;      % Lateral control D gain
% Direction control
P_direct = 0.1; % Direction control P gain
I_direct = 0; % Direction control I gain
D_direct = 0;   % Direction control D gain



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