clc
clear all
close all

%INPUTS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
input_velocity=2;  % [m/s]

discrete=1;        % 1=discrete, 0=continuous (controller and state observer)

amp_dist_roll_rate=1; 
dist_freq=100;           %rad/s

Ts=0.04;           % [sec]  %0.04 sec???????
Ts_IMU=0.01;      % [sec]  // 0.0045 in the real bike

Dead_Band=[0.055,-0.005,0.005];  % for steering motor [Dead Band Limit(+,-), Relay], [rad/sec]
max_steering_motor_acc=inf;      % maximum steering motor acceleration [rad/s^2]
min_steering_motor_acc=-inf;     % minimum steering motor acceleration [rad/s^2]
max_steering_motor_speed=7.5;    % maximum steering motor velocity [rad/s]
min_steering_motor_speed=-7.5;   % minimum steering motor velocity [rad/s]
delay=0.045;                     % delay of steering motor [sec]

complementary=0.995;             % complementary filter coefficient (0.5<c<1)
roll_rate_coef=0.8;             % coefficient of the low pass filter at the gyro output

noise_hall=0;                    % variance of hall sensor noise
noise_encoder=1e-7;              % variance of steering motor encoder noise
noise_gyro=0.001929307*(pi/180); % variance of gyroscope noise
noise_acc=0.006*(pi/180);        % variance of noise of roll angle estimation based on accelerometer
noise_roll=0.000068265*(pi/180); % variance of roll angle noise (total noise on roll angle
                                 % if complementary filter is not being used)
                                  
max_roll= pi/2;                  % maximum roll angle permitted [rad] // otherwise simulation stops
max_handlebar=40*(pi/180);       % maximum handlebar angle [rad] before the saturation
min_roll= - max_roll;            % minimum roll angle permitted [rad] // otherwise simulation stops
min_handlebar=-max_handlebar;    % minumum handlebar angle [rad] before the saturation

h = 0.586;       % height of center of mass [m]
b = 1.095;       % length between wheel centers [m]
c = 0.2;
a = b-0.77;      % distance from rear wheel to center of mass [m]
IMU_height=0.9;  % IMU height [m]    0.96 vs 0.90 vs 0.45

Q = diag([100,100,10]); % cost of states in LQR controller
R = 10;                 % cost of inputs in LQR controller  // 10 (old value), 1000 is latest value decided by Umur

plots=[0,0,0]; % Set to 1 to see the plots [eigenvalue plots (con, dis), path plot]
radius=20;     % [m]
x_ini_diff=0;
y_ini_diff=0.0;
path=1;      % 1: Straight Path --   2: Circle       3:_/-       4: ___O___
             % 5: Straight Path with Oscillations 1 /
             % 6: Straight Path with Oscillations 2 /
             % 7: Straight Path /                    8: Sinusoidal Path
             % 9: Increasing Oscillating Sinusoidal Path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     

g = 9.81;   % gravity
parameters = [g h b a];
C = eye(3);      %does not depent on velocity
D = zeros(3,1);  %does not depent on velocity

%%% LQR Gain Coefficients
velocity_lqr = linspace(0.001,10,100); % velocity_lqr = linspace(0.001,10,1000), reduced to increase speed of init
K_full=zeros(length(velocity_lqr),3);
K_full_dis=zeros(length(velocity_lqr),3);          %to be used in real bike
L_full=zeros(length(velocity_lqr),3,3);
L_full_dis=zeros(length(velocity_lqr),3,3);        %to be used in real bike
A_full=zeros(length(velocity_lqr),3,3);
A_full_dis=zeros(length(velocity_lqr),3,3);
B_full=zeros(length(velocity_lqr),3);
B_full_dis=zeros(length(velocity_lqr),3);
eigenvalues_con=zeros(3,length(velocity_lqr));
eigenvalues_dis=zeros(3,length(velocity_lqr));

for i=1:length(velocity_lqr)
    v=velocity_lqr(i);

    A = [0      0            1;
         0      0            0;
         g/h    -v^2/(h*b)   0];
    B = [0; 1; -a*v/(h*b)];
    sys_con = ss(A,B,C,D);
    sys_dis = c2d(sys_con,Ts);

    K = lqr(A, B, Q, R);
    K_full(i,:)= K;
    K_dis = lqrd(A,B,Q,R,Ts);                      %to be used in real bike
    K_full_dis(i,:)= K_dis;                        %to be used in real bike
    
    eigenvalues_con(:,i)=eig(A-B*K);
    eigenvalues_dis(:,i)=eig(sys_dis.A-sys_dis.B*K_dis);
    
    Qn=0.1;
    Rn=diag([1 1 1]);
    H=zeros(3,1);
    G=ones(3,1);
    N=0;
    sys=ss(A,[B G],C,[D H]);
    [kest,L,P] = kalman(sys,Qn,Rn);
    L_full(i,:,:)= L;
    
    sysc=ss(A,B,C,D);
    sysd=c2d(sysc,Ts);
    A_full(i,:,:)= A;
    A_full_dis(i,:,:)= sysd.A;
    B_full(i,:)= B'; 
    B_full_dis(i,:)= sysd.B';
    
%    poles=eig(sysd.A-sysd.B*K_dis);
%    L_dis = place(sysd.A',sysd.C',0.1*poles).';
    
   [kest_dis,L_dis,P_dis,M,Z]=kalmd(sys,Qn,Rn,Ts_IMU);%to be used in real bike
   L_full_dis(i,:,:)= L_dis;                      %to be used in real bike
end

L11=L_full_dis(:,1,1);
L12=L_full_dis(:,1,2);
L13=L_full_dis(:,1,3);
L21=L_full_dis(:,2,1);
L22=L_full_dis(:,2,2);
L23=L_full_dis(:,2,3);
L31=L_full_dis(:,3,1);
L32=L_full_dis(:,3,2);
L33=L_full_dis(:,3,3);

A11=A_full_dis(:,1,1);
A12=A_full_dis(:,1,2);
A13=A_full_dis(:,1,3);
A21=A_full_dis(:,2,1);
A22=A_full_dis(:,2,2);
A23=A_full_dis(:,2,3);
A31=A_full_dis(:,3,1);
A32=A_full_dis(:,3,2);
A33=A_full_dis(:,3,3);

B1=B_full_dis(:,1);
B2=B_full_dis(:,2);
B3=B_full_dis(:,3);

if plots(1)==1
    plot(velocity_lqr,real(eigenvalues_con(1,:)))
    hold on
    plot(velocity_lqr,real(eigenvalues_con(2,:)))
    plot(velocity_lqr,real(eigenvalues_con(3,:)))
    grid on
end

if plots(2)==1
    figure()
    plot(velocity_lqr,abs(eigenvalues_dis(1,:)))
    hold on
    plot(velocity_lqr,abs(eigenvalues_dis(2,:)))
    plot(velocity_lqr,abs(eigenvalues_dis(3,:)))
    grid on
end

states(1,:)=zeros(1,3);

v=input_velocity;

% Trajectory Tracking

step_time = Ts;
total_time = 5000;
time_array = 0:step_time:total_time-1;
length=size(time_array,2);

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
        path_y(round(length/3):round(2*length/3)) = 1*path_x(1:round(length/3));
        path_y(round(2*length/3)+1:end)=path_y(round(2*length/3));
        initial_x=x_ini_diff;
        initial_y=y_ini_diff;  
    
    case 4
        %___O___ ==========================================================
        path_x1 = fliplr(-time_array(1:2000));
        path_y1 = 0*path_x1+radius;
        path_x2 = radius * sin(2 * time_array(2:9425) / 10);
        path_y2 = radius * cos(2 * time_array(2:9425) / 10);
        path_x3 = time_array(1:6000);
        path_y3 = 0*path_x3+radius;
        path_x=[path_x1 path_x2 path_x3];
        path_y=[path_y1 path_y2 path_y3];
        initial_y=radius+x_ini_diff;
        initial_x=path_x(1)+y_ini_diff; 

    case 5
        % Straight Path with Oscillations 1 / =============================
        path_x = time_array;
        path_y = 0.2 * path_x + sin(path_x*.15); % INTERESTING
        initial_x=x_ini_diff;
        initial_y=y_ini_diff; 

    case 6
        % Straight Path with Oscillations 2 / =============================
        path_x = time_array;
        path_y = 0.2 * (path_x + 8 * sin(path_x*0.25)); % BAD
        initial_x=x_ini_diff;
        initial_y=y_ini_diff; 
        
    case 7
        % Straight Path / =================================================
        path_x = time_array;
        path_y = path_x; % BAD
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
end

% Calculate cumulative distance along path. 
% Adapted from https://blogs.mathworks.com/steve/2012/07/06/walking-along-a-path/
xy = [path_x' path_y'];
d = diff(xy,1);
dist_from_vertex_to_vertex = hypot(d(:,1), d(:,2));
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
        curvature(i+1)==0;
    end
    
    curvature(i+1)=abs(curvature(i+1));
    
end

curvature(1)=curvature(2);
curvature(size(path_x,2))=curvature(size(path_x,2)-1);

%% TRAJECTORY TRACKING PLOTS
close all

Bike_Path_vs_Reference_Path=1; %Set to 1 to see bike path vs reference path
Bike_Path_Error_Inscpection=0; %Set to 1 to see bike path errors 

% Bike Position vs Ideal Path
if Bike_Path_vs_Reference_Path==1
    figure(1)
    hold on
    xlabel('x [m]')
    ylabel('y [m]')
    title('Reference Path vs Bike Path')
    plot(X_SIGMA.data, Y_SIGMA.data, '*b','LineWidth', 2)
    plot(X_BIKE.data, Y_BIKE.data, '.r', 'LineWidth', 1)
    plot(initial_x,initial_y,'*r')
    legend('Reference Path', 'Bike Path', 'Starting Position')
    hold off
    grid on
end

% Bike Position vs Ideal Path with Error
if Bike_Path_Error_Inscpection==1
    figure()
    subplot(4,1,1)
    plot(X_SIGMA, '.b')
    hold on
    grid on
    plot(X_BIKE, 'r')
    title('Reference Path vs Bike Path in X Axis')
    legend('Reference Path in X','Bike Path in X')
    ylabel('Bike Path [m]')
    subplot(4,1,2)
    plot(Y_SIGMA, '.b')
    hold on
    grid on
    plot(Y_BIKE, 'r')
    title('Reference Path vs Bike Path in Y Axis')
    legend('Reference Path in Y','Bike Path in Y')
    ylabel('Bike Path [m]')
    subplot(4,1,3)
    plot(LAT_ERR)
    title('Lateral Error')
    grid on
    ylabel('Lateral Error [m]')
    xlabel('')
    subplot(4,1,4)
    plot(PSI_S-(PSI+BETA))
    grid on
    title('Angle Error')
    ylabel('Angle Error [Rad]')
end

%% COMPARE WITH CSV

% Select File
[fileName, filePath] = uigetfile( '*.csv', 'Select CSV file' );
if isequal( fileName, 0 )
return;
end
fileName = fullfile( filePath, fileName );
if ~ischar( fileName )
error( 'csvimport:FileNameError', 'The first argument to %s must be a valid .csv file', ...
  mfilename );
end

% Setup default values
p.delimiter       = ',';
p.columns         = [];
p.outputAsChar    = false;
p.uniformOutput   = true;
p.noHeader        = false;
p.ignoreWSpace    = false;

% Open file
[fid, msg] = fopen( fileName, 'rt' );
if fid == -1
  error( 'csvimport:FileReadError', 'Failed to open ''%s'' for reading.\nError Message: %s', ...
    fileName, msg );
end


% Read first line and determine number of columns in data
rowData         = fgetl( fid );
rowData         = fgetl( fid );
rowData         = regexp( rowData, p.delimiter, 'split' );
if contains('States', rowData) % is this and old CSV?
    index = find(contains(rowData, 'States'));
    rowData = [rowData(1:index-1),'Phi', 'Delta', 'Phi Dot', ...
        rowData(index+1:end)]; % rename states vector into the three states
end
dataLabels = rowData;
nCols           = numel( rowData );

%Calculate number of lines
pos             = ftell( fid );
if pos == -1
  msg = ferror( fid );
  fclose( fid );
  error( 'csvimport:FileQueryError', 'FTELL on file ''%s'' failed.\nError Message: %s', ...
    fileName, msg );
end
data            = fread( fid );
nLines          = numel( find( data == newline ) ) + 1;

%Reposition file position indicator to beginning of second line
if fseek( fid, pos, 'bof' ) ~= 0
  msg = ferror( fid );
  fclose( fid );
  error( 'csvimport:FileSeekError', 'FSEEK on file ''%s'' failed.\nError Message: %s', ...
    fileName, msg );
end

data            = cell( nLines, nCols );
data(1,:)       = rowData;
emptyRowsIdx    = [];

% Get data for remaining rows
for ii = 2 : nLines
  rowData       = fgetl( fid );
  if isempty( rowData )
    emptyRowsIdx = [emptyRowsIdx(:); ii];
    continue
  end
  rowData       = erase(rowData,["[","]"]);
  rowData       = strrep(rowData, '"', '');
  rowData       = regexp( rowData, p.delimiter, 'split' );
  nDataElems    = numel( rowData );
  if nDataElems < nCols
    warning( 'csvimport:UnevenColumns', ['Number of data elements on line %d (%d) differs from ' ...
      'that on the first line (%d). Data in this line will be padded.'], ii, nDataElems, nCols );
    rowData(nDataElems+1:nCols) = {''};
  elseif nDataElems > nCols
    warning( 'csvimport:UnevenColumns', ['Number of data elements on line %d (%d) differs from ' ...
      'that one the first line (%d). Data in this line will be truncated.'], ii, nDataElems, nCols );
    rowData     = rowData(1:nCols);
  end
  data(ii,:)    = rowData;
end

% Close file handle
fclose( fid );
data(emptyRowsIdx,:)   = [];

% Create data matrix for future processing
dataMatrix = str2double(data((2:end),:));

% Plot results
% for i = 2:size(data,2)
%     
%     figure(i)
%     plot( str2double(data(2:end,1)), str2double(data(2:end,i)), 'LineWidth', 2 )
%     xlabel(data(1,1))
%     ylabel(data(1,i))
%     legend(data(1,i))
%     grid on
% end

% All on same plot
figure
title(erase(fileName, 'C:\Users\Boaz Ash\Google Drive\Bike Project\Modelling and Control\Results\Balance Tests\'))

for i = 2:size(data,2)
    subplot(2,3,i-1)
    plot( str2double(data(2:end,1)), str2double(data(2:end,i)), 'LineWidth', 2 )
    xlabel(data(1,1))
    ylabel(data(1,i))
    legend(data(1,i))
    grid on
end

figure
hold on
plot(simulation_delta_ref, 'r', 'LineWidth', 2)
plot(simulation_delta, 'b', 'LineWidth', 2)
plot(str2double(data(2:end,1)), str2double(data(2:end,6)), 'g', 'LineWidth', 2)
hold off
grid on
xlabel('Time (s)')
ylabel('Steering angle (rad)')
legend('Reference Delta', 'Simulation Delta', 'Measured Bike Delta');
title('Model vs Bike Steering Step Response (Step = -PI/6)');
