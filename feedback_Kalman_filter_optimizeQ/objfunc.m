function cost = objfunc(XQ)
load('gyro_noise.mat')
load('acc_y_noise.mat')

%==========================================================================
    %IMU separation from bike model implemented
    %Real/Model parameters distinction
    %3D simulation implemented
    %Implementation of roll angle saturation for 3D simulation
    %Oscillations 
    
    %DO NOT CONTAIN
    %Comparison model and real bike -> statistic results on inaccurate
    %model

%NEW
    %Position feedback loop implementation with KALMAN FILTER
    %Path tracking performance indicator NOT READY
    %Uses : bike_model_v1_20190510_oldmodel_positionfeedback.slx

%==========================================================================

%% Parameters Initialization
%INPUTS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIMULATION / INITIALIZATION
sim_time=74;       % [s] Simulation time : Max: 89 sec (due to available noise data)
initial_states=[deg2rad(0) deg2rad(0) deg2rad(0)]; % Initial states of the bike at t=0
                   % [roll angle in rad, steering angle in rad, roll rate in rad/s];
                     
% VELOCITY
input_velocity=3;            % [m/s]   forward velocity
velocity_wave_amp=0.5;         % [m/s]   amplitude of sinusoidal wave summped up with forward velocity
velocity_wave_freq=0.3*2*pi;   % [rad/s] frequency of sinusoidal wave summped up with forward velocity
min_max_velocity=[input_velocity input_velocity]; % [m/s] If variable
                   % velocity is used in the simulation, min and max values
                   % of the reference velocity profile should be specified
                   % here for calculating corresponding dynamics.

% NOISE / LOOK AHEAD
noise=1;           % 1=enable noise on the sensors, 0=disable noise
look_ahead=0;      % 1=enable look-ahead path control, 0=disable
                   % EXPERIMENTAL, NOT FINALIZED!


% DYNAMICS / PATH TRACKING / MODEL
oscillations_roll = 0;        %1=enable oscillation on steering due to the box, 0=disable
oscillations_steer = 0;
    damp = 0.4;
    fr_oscil = 1;        %frequency in Hz
    gain = 0.3;
lat_control=1;      %1=enable lateral control, 0=disable
direct_control=1;   %1=enable direction control, 0=disable
delta_control=1;    %1=enable steering angle (delta) control, 0=disable

fork_dynamics_lqr=1;
forward_motor_dynamics=0;     % 1=enable forward motor dynamics, 0=disable
steering_motor_limitations=1; % 1=enable limitations(deadband, delay),0=disable
path_tracking=1;  % 1=path tracking, 0=only self balancing, no path tracking
path_tracking_indicator=0;

plant_model=1;    % 1=nonlinear plant, 2=linear plant,
                  % 3=linear plant with neglected fork angle(the simplest plant)
filter_model=1; % 1=use measurements for compensation
                % 2=use model to estimate the necessary values for compensation 
                %(with IC, but not with the recent measurement)
                % 3=use model to estimate the necessary values for compensation 
                %(with IC, and then iterate autonomously)

% SAMPLING TIME
Ts=0.04;          % [sec]  %0.04 sec
Ts_IMU=0.04;      % [sec]  // 0.0045 in the real bike

% STEERING MOTOR LIMITATIONS
Dead_Band=[0.055,-0.055,0.005];  % for steering motor[Dead Band Limit(+,-), Relay], 
                                 %[rad/sec]
max_steering_motor_acc=inf;      % maximum steering motor acceleration [rad/s^2]
min_steering_motor_acc=-inf;     % minimum steering motor acceleration [rad/s^2]
max_steering_motor_speed=7.5;    % maximum steering motor velocity [rad/s]
min_steering_motor_speed=-7.5;   % minimum steering motor velocity [rad/s]
delay=0.045;                     % delay of steering motor [sec]

% COEFFICIENT FILTERS
complementary=0.985;%0.995       % complementary filter coefficient (0.5<c<1) 
roll_rate_coef=0.9;%0.8          % coefficient of the low pass filter at the gyro 
                                 % output 

% NOISE
noise_hall=0.0001;    %0           % variance of hall sensor noise 
noise_encoder=1e-7;              % variance of steering motor encoder noise
noise_gyro=0.001929307*(pi/180); % variance of gyroscope noise
noise_acc=0.006*(pi/180);        % variance of noise of roll angle estimation 
                                 % based on accelerometer
noise_pos_meas=0.51;   %or 9     % variance of position measurement system

%noise_roll=0.000068265*(pi/180); % variance of roll angle noise (total noise 
                                  % on roll angle if complementary filter is not being used)
% KALMAN FILTER - POSITION ESTIMATION
A_kalman=[1 0 Ts 0; 0 1 0 Ts; 0 0 1 0; 0 0 0 1];
B_kalman=[0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
C_kalman=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
D_kalman=[0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];

% Q_kalman = [Q11 0 0 0 ;0 Q22 0 0; 0 0 Q33 0; 0 0 0 Q44];                 % Process noise covariance
Q_kalman = diag(XQ);
R_kalman = [noise_pos_meas 0 0 0; 0 noise_pos_meas 0 0; 0 0 noise_hall 0; 0 0 0 noise_hall];    % Measurement noise covariance

% ANGLES LIMITATIONS
max_roll= pi/2;                  % maximum roll angle permitted [rad] 
                                 % // otherwise simulation stops
max_handlebar=40*(pi/180);       % maximum handlebar angle [rad] before the saturation
min_roll= - max_roll;            % minimum roll angle permitted [rad] 
                                 % // otherwise simulation stops
min_handlebar=-max_handlebar;    % minumum handlebar angle [rad] before the saturation

% REAL BIKE PARAMETERS
h_real = 0.586;           % height of center of mass [m]
b_real = 1.095;           % length between wheel centers [m]
c_real = 0.06;            % length between front wheel contact point and the 
                          % extention of the fork axis
lambda_real=deg2rad(70);  % angle of the fork axis
a_real = b_real-0.77;     % distance from rear wheel to center of mass [m]
IMU_height_real=0.45;     % IMU height [m]    0.96 vs 0.90 vs 0.45
g = 9.81;                 % gravity

parameters_real = [g h_real b_real a_real lambda_real c_real];

% HALLSENSOR PARAMETERS
dout = 0.7; % The diameter of the outer tyre frame
nrsensor = 5; % The number or magnets in hall sensor
sensordist = dout*pi/nrsensor; % The distances between different magnets
ToleranceVelocityEst = 0.5; % The tolerance for forward velocity filter

% MODEL BIKE PARAMETERS
h_model = 0.586;          % height of center of mass [m]
b_model = 1.095;          % length between wheel centers [m]
c_model = 0.06;           % length between front wheel contact point and the 
                          % extention of the fork axis
lambda_model=deg2rad(70); % angle of the fork axis
a_model = b_model-0.77;   % distance from rear wheel to center of mass [m]
IMU_height_model=0.45;    % IMU height [m]    0.96 vs 0.90 vs 0.45

parameters_model = [g h_model b_model a_model lambda_model c_model];

% PLOT SETTINGS
plots=[0,0,0];   % Set to 1 to see the plots [eigenvalue plot, K plot, path plot]
radius=20;       % [m]
x_ini_diff=0;
y_ini_diff=0;

slope=0.3;   % slope for path 3 , 5 and 6
path=1;      % 1: Straight Path --   2: Circle       3:_/-       4: ___O___
             % 5: Straight Path with Oscillations 1 /
             % 6: Straight Path with Oscillations 2 /
             % 7: Straight Path /                    8: Sinusoidal Path
             % 9: Increasing Oscillating Sinusoidal Path
             %10: 45? curve (set radius)
             %11: 90? curve (set radius)
             %12: Straight, turn along curve, straight back (set radius)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% LQR CONTROLLER PARAMETERS
% Yixiao
    Q = diag([100,100,10]); % cost of states in LQR controller
    R = 42;                 % cost of inputs in LQR controller  
% Umur
    %Q = diag([10000,10,100]); % cost of states in LQR controller 
    %R = 30;                 % cost of inputs in LQR controller 

%%% LQR Gain Coefficients
C = eye(3);      %does not depent on velocity
D = zeros(3,1);  %does not depent on velocity

velocity_lqr = min_max_velocity(1):0.01:min_max_velocity(2);

if size(velocity_lqr)==[1 1]
    velocity_lqr=[velocity_lqr-0.1 velocity_lqr velocity_lqr+0.1];
end

K_full_dis=zeros(length(velocity_lqr),3);
eigenvalues_dis=zeros(3,length(velocity_lqr));

if fork_dynamics_lqr==1

    for i=1:length(velocity_lqr)
        v=velocity_lqr(i);

        A = [0      0            1;
             0      0            0;
             g/h_real    sin(lambda_real)*(h_real*v^2-g*a_real*c_real)/(h_real^2*b_real)   0];
        B = [0; 1; (a_real*v*sin(lambda_real))/(h_real*b_real)];
        sys_con = ss(A,B,C,D);
        sys_dis = c2d(sys_con,Ts);

        K_dis = lqrd(A,B,Q,R,Ts);
        K_full_dis(i,:)= K_dis;

        sysc=ss(A,B,C,D);
        sysd=c2d(sysc,Ts); 
    end
    
else
    
    for i=1:length(velocity_lqr)
        v=velocity_lqr(i);

        A = [0      0            1;
             0      0            0;
             g/h_real    v^2/(h_real*b_real)   0];
        B = [0; 1; a_real*v/(h_real*b_real)];
        sys_con = ss(A,B,C,D);
        sys_dis = c2d(sys_con,Ts);

        K_dis = lqrd(A,B,Q,R,Ts);
        K_full_dis(i,:)= K_dis;

        sysc=ss(A,B,C,D);
        sysd=c2d(sysc,Ts); 
    end
    
end
    

if plots(1)==1
    
    velocity_eig = linspace(0.001,10,100); % velocity_lqr = linspace(0.001,10,1000), 
                                           % reduced to increase speed of init
    K_full_dis_eig=zeros(length(velocity_eig),3);    %to be used in real bike
    eigenvalues_dis_eig=zeros(3,length(velocity_eig));

    for i=1:length(velocity_eig)
        v=velocity_eig(i);

        A = [0      0            1;
             0      0            0;
             g/h_real    sin(lambda_real)*(h_real*v^2-g*a_real*c_real)/(h_real^2*b_real)   0];
        B = [0; 1; (a_real*v*sin(lambda_real))/(h_real*b_real)];

        sys_con = ss(A,B,C,D);
        sys_dis = c2d(sys_con,Ts);

        K_dis_eig = lqrd(A,B,Q,R,Ts);     
        K_full_dis_eig(i,:)= K_dis_eig;

        eigenvalues_dis_eig(:,i)=eig(sys_dis.A-sys_dis.B*K_dis);
    end
    
    eigenvalues_abs=abs(eigenvalues_dis_eig);
    eigenvalues_abs = sort(eigenvalues_abs,1);
    
    figure()
    plot(velocity_eig,eigenvalues_abs(1,:))
    hold on
    plot(velocity_eig,eigenvalues_abs(2,:))
    plot(velocity_eig,eigenvalues_abs(3,:))
    grid on
    %title('Eigenvalues of the Closed Loop System vs Veloctity [m/s]')
    xlabel('Velocity [m/s]')
    ylabel('Magnitude of the Eigenvalues')
    
end


if plots(2)==1
    
    velocity_K = linspace(1.5,10,1000); % velocity_lqr = linspace(0.001,10,1000), 
                                        % reduced to increase speed of init
    K_full_dis_eig_plot=zeros(length(velocity_K),3); %to be used in real bike
    eigenvalues_dis_eig=zeros(3,length(velocity_K));

    for i=1:length(velocity_K)
        v=velocity_K(i);

        A = [0      0            1;
             0      0            0;
             g/h_real    sin(lambda_real)*(h_real*v^2-g*a_real*c_real)/(h_real^2*b_real)   0];
        B = [0; 1; (a_real*v*sin(lambda_real))/(h_real*b_real)];

        sys_con = ss(A,B,C,D);
        sys_dis = c2d(sys_con,Ts);

        K_dis_plot = lqrd(A,B,Q,R,Ts);     
        K_full_dis_eig_plot(i,:)= K_dis_plot;

    end
    
    figure()
    subplot(3,1,1)
    plot(velocity_K,K_full_dis_eig_plot(:,1))
    ylabel('$K_\varphi$','interpreter','latex')
    grid on
    subplot(3,1,2)
    plot(velocity_K,K_full_dis_eig_plot(:,2))
    ylabel('$K_\delta$','interpreter','latex')
    grid on
    subplot(3,1,3)
    plot(velocity_K,K_full_dis_eig_plot(:,3))
    xlabel('Velocity [m/s]')
    ylabel('$K_{\dot{\varphi}}$','interpreter','latex')
    grid on
    
end

states(1,:)=zeros(1,3);

v=input_velocity;

% Trajectory Tracking

step_time = Ts;
total_time = sim_time*5;
time_array = 0:step_time:total_time;
length1=size(time_array,2);

switch path
    
    case 1
        %Straight Path -- =================================================
        path_x = time_array;
        path_y = 0*path_x;
        initial_x=x_ini_diff;
        initial_y=y_ini_diff;   
    
    case 2 %Change the starting point to (r,0) and direction CCW to make it similar to PYTHON
        %Circle ===========================================================
        path_x = radius * sin(2 * time_array / 10);
        path_y = radius * cos(2 * time_array / 10);
        initial_y=radius+y_ini_diff;
        initial_x=0;
        
        
    case 3
        %_/- ==============================================================
        path_x = time_array;
        path_y = 0*path_x;
        path_y(round(length1/3):round(2*length1/3)) = slope*path_x(1:round(length1/3));
        path_y(round(2*length1/3)+1:end)=path_y(round(2*length1/3));
        initial_x=x_ini_diff;
        initial_y=y_ini_diff;  
    
    case 4
        %___O___ ==========================================================

        path_x1=linspace(-40,-0.05,100);
        path_y1=radius*ones(size(path_x1));
        path_x3=linspace(0.05,40,100);
        path_y3=radius*ones(size(path_x3));
        
        angle2=linspace(0,2*pi,100);
        path_x2 = radius * sin(angle2);
        path_y2 = radius * cos(angle2);        
        path_x=[path_x1 path_x2 path_x3];
        path_y=[path_y1 path_y2 path_y3];
        initial_y=radius+y_ini_diff;
        initial_x=path_x(1)+x_ini_diff; 

    case 5
        % Straight Path with Oscillations 1 / =============================
        path_x = time_array;
        path_y = slope * path_x + sin(path_x*.1); % INTERESTING
        initial_x=x_ini_diff;
        initial_y=y_ini_diff; 

    case 6
        % Straight Path with Oscillations 2 / =============================
        path_x = time_array;
        path_y = slope * (path_x + 6 * sin(path_x*0.20)); % BAD
        initial_x=x_ini_diff;
        initial_y=y_ini_diff; 
        
    case 7
        % Straight Path / =================================================
        path_x = time_array;
        path_y = 0.2*path_x; % BAD
        initial_x=x_ini_diff;
        initial_y=y_ini_diff;         
        
    case 8
        % Sinusoidal Path =================================================
        path_x = time_array;
        path_y = 4 * sin((path_x*2*pi)/100); % GOOD
        initial_x=x_ini_diff;
        initial_y=y_ini_diff; 

    case 9
        %Increasing Oscillating Sinusoidal Path ===========================
        path_x = time_array;
        path_y = sqrt(path_x * 0.8) .* sin(0.1*path_x); % GOOD
        initial_x=x_ini_diff;
        initial_y=y_ini_diff; 
     case 10
        %___/ 45? curve ==========================================================
        path_x1=linspace(-40,-0.05,100);
        path_y1=radius*ones(size(path_x1));
        path_x3=linspace(0.5*sqrt(2)*radius,80,100);
        path_y3=linspace((0.5*sqrt(2)*radius),-60,100);
                
        angle2=linspace(0,0.25*pi,100);
        path_x2 = radius * sin(angle2);
        path_y2 = radius * cos(angle2);        
        path_x=[path_x1 path_x2 path_x3(2:length(path_x3))];
        path_y=[path_y1 path_y2 path_y3(2:length(path_y3))];
        initial_y=radius+y_ini_diff;
        initial_x=path_x(1)+x_ini_diff;
        
    case 11
        %___| 90? curve ==========================================================
        path_x1=linspace(-40,-0.05,100);
        path_y1=radius*ones(size(path_x1));
        path_y3=linspace(-0.05,-80,100);
        path_x3=radius*ones(size(path_y3));
        
        angle2=linspace(0,0.5*pi,100);
        path_x2 = radius * sin(angle2);
        path_y2 = radius * cos(angle2);        
        path_x=[path_x1 path_x2 path_x3];
        path_y=[path_y1 path_y2 path_y3];
        initial_y=radius+y_ini_diff;
        initial_x=path_x(1)+x_ini_diff; 
        
    case 12
        %way turn and back ==========================================================        
        path_x1=linspace(-40,-0.05,100);
        path_y1=radius*ones(size(path_x1));
        path_x3=linspace(-0.05,-80,100);
        path_y3=-radius*ones(size(path_x3));
        
        angle2=linspace(0,pi,100);
        path_x2 = radius * sin(angle2);
        path_y2 = radius * cos(angle2);        
        path_x=[path_x1 path_x2 path_x3];
        path_y=[path_y1 path_y2 path_y3];
        initial_y=radius+y_ini_diff;
        initial_x=path_x(1)+x_ini_diff; 
end

% Calculate cumulative distance along path. 
% Adapted from https://blogs.mathworks.com/steve/2012/07/06/walking-along-a-path/
xy = [path_x' path_y'];
d = diff(xy,1);
dist_from_vertex_to_vertex = hypot(d(:,1), d(:,2)); % C = hypot(A,B) -> C = sqrt(abs(A).^2 + abs(B).^2)
cumulative_dist_along_path = [0;cumsum(dist_from_vertex_to_vertex,1)];

if plots(3)==1
    figure()
    plot(path_x,path_y, '.');
    title('x vs y')
    xlabel('x')
    ylabel('y')
    axis([min(path_x)-5 max(path_x)+5 min(path_y)-5 max(path_y)+5])
    grid on
 end

% Radius of Curvature Calculation

for i=1:(size(path_x,2)-2)
    
    length_a=sqrt( (path_x(i+1) - path_x(i) )^2 + ( path_y(i+1) - path_y(i) )^2 );
    length_b=sqrt( (path_x(i+2) - path_x(i) )^2 + ( path_y(i+2) - path_y(i) )^2 );
    length_c=sqrt( (path_x(i+2) - path_x(i+1) )^2 + (path_y(i+2) - path_y(i+1) )^2 );
    per=(length_a+length_b+length_c)/2;
    area=sqrt( per * (per-length_a) * (per-length_b) * (per-length_c) );
    
    curvature(i+1)=(4*area)/(length_a*length_b*length_c);
    
    if curvature(i+1)<0.01
        curvature(i+1)=0;
    end
    
    curvature(i+1)=abs(curvature(i+1));
    
end

curvature(1)=curvature(2);
curvature(size(path_x,2))=curvature(size(path_x,2)-1);

%% RUN THE SIMULINK SIMULATION

% try
    assignin('base','Q_kalman',Q_kalman);
    sim('bike_model_v1_20190510_oldmodel_positionfeedback.slx')
    cost = sum(sqrt((X_SIGMA.data-X_BIKE.data).^2 + (Y_SIGMA.data-Y_BIKE.data).^2));
% catch Error_Reason
%     cost = 1e9;
% end


end