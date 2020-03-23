%% Simulink

% Simulink file
simulink_file = 'bike_model.slx';

% Simulation time
sim_time = 40;  % Simulation time [s]

% Initial angles
initial_roll = 0; %rad
initial_rollrate = 0; %rad

% Sampling Time
Ts = 0.01;     % Sampling time for the measurements and control loop [s]


%% Physical bike
% Velocity
input_velocity = 4;  % Forward velocity [m/s]


% Real Bike Parameters
r_wheel = 0.311; % Radius of the wheel
h_real = 0.2085 + r_wheel; % height of center of mass [m]
b_real = 1.095;            % length between wheel centers [m]
c_real = 0.06;             % length between front wheel contact point and the 
                           % extention of the fork axis [m]
lambda_real = deg2rad(70); % angle of the fork axis [deg]
a_real = 0.4964;           % distance from rear wheel to frame's center of mass [m]
IMU_height_real = 0.45;    % IMU height [m]
g = 9.81;                  % gravity [m/s^2]


% Store bike parameters in a vector
parameters_compFilter = [g h_real b_real a_real lambda_real c_real];


%% State-space matrices
A_bike = [0 g/h_real ; 1 0];
B_bike = [1 ; 0];
C1_bike = a_real/(b_real*h_real); % To be multiplied by v in Simulink
C2_bike = 1/(b_real*h_real); % To be multiplied by v^2 in Simulink
D_bike = 0;

[n,m] = size(A_bike);

sys_bike = ss(A_bike,B_bike,eye(n),0);

C_ini = [C1_bike*input_velocity C2_bike*input_velocity^2];


%% LQR control
% LQR weights
% Q and R weight matrices in phi and phi_dot
% The matrices are transformed to states x1 and x2 in the lqr_design script
Q_phi = [10 -9 ; -9 10];
R_phi = 1;