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
bike_params = [g h b a lambda m];


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


%% Analysis of the open-loop bike model
disp('Poles of the bike in continuous time');
disp(eig(A));

disp('Poles of the bike in discrete time');
disp(eig(A_dis));

if plots
    figure;bode(sys_tf);
    figure;margin(sys_tf);
end


%% LQR design
% Q and R weight matrices in phi and phi_dot
% The matrices are transformed to states x1 and x2 in the lqr_design script
% Slow LQR
% Q_phi = [10 -9 ; -9 10];
% R_phi = 1;

% Medium LQR
Q_phi = [10 -3 ; -3 1];
R_phi = 1;

% Fast LQR
% Q_phi = [100 -9 ; -9 1];
% R_phi = 1;

% Very Fast LQR
% Q_phi = [1000 -3 ; -3 1];
% R_phi = 1;

% Very slow LQR (22s rise time)
% Q_phi = [1 0 ; 0 10];
% R_phi = 1;

% Fast LQR, bandwidth ~=3.5Hz, weight on phidot = 0
% Q_phi = [10 0 ; 0 0];
% R_phi = 1;

% Medium LQR, bandwidth (from roll perturbation to true roll) ~=1Hz, weight on phidot = 0
% Q_phi = [1 0 ; 0 0];
% R_phi = 1;

% Medium LQR, bandwidth (from roll perturbation to true roll) ~=0.7Hz, weight on phidot = 0
% Q_phi = [0.4 0 ; 0 0];
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
disp('Poles of the closed-loop balanced bike (bike+LQR) in discrete time');
disp(eig(sys_dis_LQR.A));
disp('Poles of the closed-loop balanced bike (bike+LQR) in continuous time');
disp(eig(d2c(sys_dis_LQR)));

if plots
    figure;bode(sys_dis_LQR);
    figure;margin(sys_dis_LQR);
end


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

if plots
    figure;bode(sys_tf_steering_dis);
    figure;margin(sys_tf_steering_dis);
end