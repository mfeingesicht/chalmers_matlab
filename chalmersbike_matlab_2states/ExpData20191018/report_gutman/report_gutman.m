clear all
close all
clc


%% Bike parameters
% Initial angles
initial_roll = deg2rad(2);
initial_rollrate = deg2rad(0);
initial_steering = deg2rad(0);

% Sampling time
Ts = 0.01;            % sampling time

% Forward velocity
% v = 3;                % velocity [m/s]
v = 4;                % velocity [m/s]
% v = 5;                % velocity [m/s]

% Bike physical parameters
r_wheel = 0.311;      % radius of the wheel
h = 0.2085 + r_wheel; % height of center of mass [m]
b = 1.095;            % length between wheel centers [m]
c = 0.06;             % length between front wheel contact point and the extention of the fork axis [m]
lambda = deg2rad(70); % angle of the fork axis [deg]
a = 0.4964;           % distance from rear wheel to frame's center of mass [m]
IMU_height = 0.45;    % IMU height [m]
g = 9.81;             % gravity [m/s^2]
m = 45;               % Bike mas [kg]

% Complementary filter
complementary = 0.985;
roll_rate_coef = 0.9;

% Noise and offsets for sensors and actuators
forward_motor_dynamics = 0;
noise = 0;
sensor_offset = 0;
noise_hall = 0;
forward_speed_offset = 0;
time_offset_hall = 0;
offset_hall = 0;
noise_acc = 0;
noise_pos_meas = 0;
noise_encoder = 0;
noise_gyro = 0;
time_offset_pos_x = 0;
steering_angle_correction = 0;
time_offset_acc = 0;
offset_pos_x = 0;
time_offset_encoder = 0;
offset_acc = 0;
time_offset_pos_y = 0;
offset_encoder = 0;
time_offset_gyro = 0;
offset_pos_y = 0;
offset_gyro = 0;


%% Bike transfer function
num = -[v*a v^2];
den = [-b*h 0 b*g];
sys_tf = tf(num,den);


%% Bike state-space matrices
% Continuous time
A = [0 g/h ; 1 0];
B = [1 ; 0];
C1 = a/(b*h); % To be multiplied by v in Simulink
C2 = 1/(b*h); % To be multiplied by v^2 in Simulink
C = [C1*v C2*v^2];
D = 0;

% [n,m] = size(A);

sys_fullstates = ss(A,B,eye(size(A)),0);
sys = ss(A,B,C,D);


% Discrete time
sys_dis = c2d(sys,Ts);
A_dis = sys_dis.A;
B_dis = sys_dis.B;
C_dis = sys_dis.C;
D_dis = sys_dis.D;


%% Analysis of the open-loop bike model
disp('Poles of the bike in continuous time');
disp(eig(A));

disp('Poles of the bike in discrete time');
disp(eig(A_dis));

figure;bode(sys_tf);
figure;margin(sys_tf);


%% LQR design
% Q and R weight matrices in phi and phi_dot
% The matrices are transformed to states x1 and x2 in the lqr_design script
% Slow LQR
Q_phi = [10 -9 ; -9 10];
R_phi = 1;

% Medium LQR
% Q_phi = [10 -3 ; -3 1];
% R_phi = 1;

% Fast LQR
% Q_phi = [100 -9 ; -9 1];
% R_phi = 1;

lqr_design;
K_lqr = lqr_gains(2,2:end)


%% Butterworth LP filter
[bb,ba]=butter(2,1/(100/2));
% [bb,ba]=butter(1,6/(100/2));


%% Analysis of the open-loop bike model
sys_dis_LQR = ss(A_dis-B_dis*K_lqr,B_dis,C_dis,D_dis,Ts);

disp(' ');
disp(' ');
disp(' ');
disp('Poles of the closed-loop balanced bike (bike+LQR)');
disp(eig(sys_dis_LQR.A));

figure;bode(sys_dis_LQR);
figure;margin(sys_dis_LQR);


%% Steering motor
num_tf_steering = [253.4];
den_tf_steering = [1 245.9 0];
sys_tf_steering = tf(num_tf_steering,den_tf_steering);

sys_tf_steering_dis = c2d(sys_tf_steering,Ts);

P_steering_position = 30;
I_steering_position = 30;
D_steering_position = 0;
K_feedforward_steering = 0;


%% Analysis of the open-loop steering motor model
disp(' ');
disp(' ');
disp(' ');
disp('Poles of the steering motor in continuous time');
disp(roots(den_tf_steering));

disp('Poles of the steering motor in discrete time');
disp(roots(sys_tf_steering_dis.den{:}));

figure;bode(sys_tf_steering_dis);
figure;margin(sys_tf_steering_dis);


%% Inertia front
% inertia_front = 4;
inertia_front = 0.2741;
% inertia_front = 0.2741*2;
% inertia_front = 0;

% Bike transfer function
% num_front = [a*v v^2];
num_front = [(inertia_front*b)/(m*h) a*v v^2];
den_front = [b*h 0 -b*g];
sys_tf_front = tf(num_front,den_front);

% Bike state-space matrices
% Continuous time
[A_front,B_front,C_front,D_front] = tf2ss(num_front,den_front);
sys_front = ss(A_front,B_front,C_front,D_front);

% LQR design
lqr_design_front;
K_lqr = lqr_gains(2,2:end);