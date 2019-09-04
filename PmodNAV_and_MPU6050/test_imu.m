function [] = test_imu()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2017 ROBOTIS CO., LTD.
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Author: Ryu Woon Jung (Leon)

%
% *********     Read and Write Example      *********
%
%
% Available Dynamixel model on this example : All models using Protocol 2.0
% This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
% To use another Dynamixel model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
% Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
%

clc;
clear all;
close all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

% Control table address
% ADDR_PRO_TORQUE_ENABLE       = 562;         % Control table address is different in Dynamixel model
% ADDR_PRO_GOAL_POSITION       = 596;
% ADDR_PRO_PRESENT_POSITION    = 611;
ADDR_PRO_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116;
ADDR_PRO_PRESENT_POSITION    = 132;
ADDR_PRO_GOAL_VELOCITY       = 104;
ADDR_PRO_OPERATING_MODE      = 11;

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID                      = 1;            % Dynamixel ID: 1
BAUDRATE                    = 57600;
DEVICENAME                  = 'COM6';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

% OPERATING_MODE              = 3;            % Position control mode
OPERATING_MODE              = 1;            % Velocity control mode
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque

DXL_MINIMUM_POSITION_VALUE  = 2000;         % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 3660;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MIDDLE_POSITION_VALUE   = 2830;         % Upwards bar position
DXL_MOVING_STATUS_THRESHOLD = 3;            % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position

% Set goal position and velocity vectors
index_pos = 1;
index_vel = 1;

% dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MIDDLE_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE DXL_MIDDLE_POSITION_VALUE];         % Goal positions, betweeb 1800 and 3960
dxl_goal_position = [DXL_MIDDLE_POSITION_VALUE];

% % dxl_goal_velocity = [0 330 40 2];         % Goal velocities, between -330 and 330
dxl_goal_velocity = [5 -5 10 -10];         % Goal velocities, between -330 and 330
% % dxl_goal_velocity = [0];         % Goal velocities, between -330 and 330

% Choose plots
plot_acc_gyro = 0;
plot_roll = 0;

%% Configure the Motor
% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end


% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Torque successfully connected \n');
end


% Choose operating mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_OPERATING_MODE, OPERATING_MODE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Operating mode successfully set to %d (1: Velocity, 3: Position) \n',OPERATING_MODE);
end


% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Torque successfully connected \n');
end


%% Connect to the Arduino
% Create the serial object and open the port
serial_arduino = serial('COM5','BaudRate',115200,'Terminator','CR/LF');
fopen(serial_arduino);

time_loop_arduino = 0;

% % % Flush the serial buffer until good data has been receievd
% % while length(tline) ~= 149
% %     tline = fgetl(serial_arduino);
% % end

tline = {};


%% Cleanup function
c = onCleanup(@() myCleanupFunction(port_num, lib_name, PROTOCOL_VERSION,DXL_ID, ADDR_PRO_TORQUE_ENABLE, ADDR_PRO_OPERATING_MODE, ADDR_PRO_GOAL_POSITION, TORQUE_DISABLE, TORQUE_ENABLE, DXL_MIDDLE_POSITION_VALUE, COMM_SUCCESS, serial_arduino));


%% Define vectors to store data for plotting
dxl_present_position_vec = [];
dxl_present_velocity_vec = [];

roll_true_vec = [0];

% PmodNAV
ax_vec_PmodNAV = [0];
ay_vec_PmodNAV = [0];
az_vec_PmodNAV = [0];
gx_vec_PmodNAV = [0];
gy_vec_PmodNAV = [0];
gz_vec_PmodNAV = [0];
mx_vec_PmodNAV = [0];
my_vec_PmodNAV = [0];
mz_vec_PmodNAV = [0];
roll_madgwick_vec_PmodNAV = [0];
roll_mahony_vec_PmodNAV = [0];
roll_comp_vec_PmodNAV = [0];
roll_intGyro_vec_PmodNAV = [0];

% MPU6050
ax_vec_MPU6050 = [0];
ay_vec_MPU6050 = [0];
az_vec_MPU6050 = [0];
gx_vec_MPU6050 = [0];
gy_vec_MPU6050 = [0];
gz_vec_MPU6050 = [0];
roll_comp_vec_MPU6050 = [0];
roll_comp_latComp_vec_MPU6050 = [0];
roll_intGyro_vec_MPU6050 = [0];


%% Initialize time measurement
time_ini = tic;
time = [0];


%% Initialize plots
if plot_roll
    h = figure;ylim([-90 90]);
    hold on;
    h1 = plot(time,roll_comp_vec_MPU6050,'XDataSource','time','YDataSource','roll_comp_vec');
    h2 = plot(time,roll_comp_latComp_vec_MPU6050,'XDataSource','time','YDataSource','roll_comp_latComp_vec');
    h3 = plot(time,roll_intGyro_vec_MPU6050,'XDataSource','time','YDataSource','roll_intGyro_vec');
    h4 = plot(time,roll_true_vec_MPU6050,'XDataSource','time','YDataSource','roll_true_vec');
    legend('Roll complementary','Roll complementary with LatComp','Roll rate integration','True roll');
    drawnow;
end

if plot_acc_gyro
    hh = figure;
    hold on;
    subplot(321);hh1 = plot(time,ax_vec_MPU6050,'XDataSource','time','YDataSource','ax_vec');
    subplot(322);hh2 = plot(time,ay_vec_MPU6050,'XDataSource','time','YDataSource','ay_vec');
    subplot(323);hh3 = plot(time,az_vec_MPU6050,'XDataSource','time','YDataSource','az_vec');
    subplot(324);hh4 = plot(time,gx_vec_MPU6050,'XDataSource','time','YDataSource','gx_vec');
    subplot(325);hh5 = plot(time,gy_vec_MPU6050,'XDataSource','time','YDataSource','gy_vec');
    subplot(326);hh6 = plot(time,gz_vec_MPU6050,'XDataSource','time','YDataSource','gz_vec');
    % legend('Roll Madwick','Roll Mahony','Roll complementary','Roll rate integration','True roll');
end


%% Pause to give time to Arduino to startup
pause(5);


%% Loop
while 1
%     if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
%         break;
%     end
    
    %% Write position/velocity goals to the motor
    % Write goal velocity
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_VELOCITY, typecast(int32(dxl_goal_velocity(index_vel)), 'uint32'));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    % Write goal position
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl_goal_position(index_pos)), 'uint32'));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    
    while 1
        %% Read position and velocity from the motor
        % Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end
        
        % Read present velocity
        dxl_present_velocity = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, 104);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end
        
        
        %% Read data from the Arduino
        % Flush and read until good data has been received
        time_loop_arduino = tic;
        
        while 1
            flushinput(serial_arduino);
            fwrite(serial_arduino,'1');
            tline{end+1} = fgetl(serial_arduino);
            if ~isempty(tline{end})
                if tline{end}(1) == '$'
                    C = strsplit(tline{end}(2:end),{',','$'});
                    if length(C) == 35 && all(ismember(C{1},'0123456789+-.eEdD*$'))
                        disp('Good Arduino data')
                        break;
                    end
                    disp('Bad Arduino data')
                    tline(end)=[];
                end
            end
        end        
        
        % PmodNAV
        ax_PmodNAV = str2double(C{1});
        ay_PmodNAV = str2double(C{2});
        az_PmodNAV = str2double(C{3});
        gx_PmodNAV = str2double(C{4});
        gy_PmodNAV = str2double(C{5});
        gz_PmodNAV = str2double(C{6});
        mx_PmodNAV = str2double(C{7});
        my_PmodNAV = str2double(C{8});
        mz_PmodNAV = str2double(C{9});
        q_madgwick0_PmodNAV = str2double(C{10});
        q_madgwickx_PmodNAV = str2double(C{11});
        q_madgwicky_PmodNAV = str2double(C{12});
        q_madgwickz_PmodNAV = str2double(C{13});
        q_mahony0_PmodNAV = str2double(C{14});
        q_mahonyx_PmodNAV = str2double(C{15});
        q_mahonyy_PmodNAV = str2double(C{16});
        q_mahonyz_PmodNAV = str2double(C{17});
        roll_madgwick_PmodNAV = str2double(C{18});
        pitch_madgwick_PmodNAV = str2double(C{19});
        yaw_madgwick_PmodNAV = str2double(C{20});
        roll_mahony_PmodNAV = str2double(C{21});
        pitch_mahony_PmodNAV = str2double(C{22});
        yaw_mahony_PmodNAV = str2double(C{23});
        roll_comp_PmodNAV = str2double(C{24});
        roll_intGyro_PmodNAV = str2double(C{25});
        average_rate_PmodNAV = str2double(C{26});
        
        % MPU6050
        ax_MPU6050 = str2double(C{27});
        ay_MPU6050 = str2double(C{28});
        az_MPU6050 = str2double(C{29});
        gx_MPU6050 = str2double(C{30});
        gy_MPU6050 = str2double(C{31});
        gz_MPU6050 = str2double(C{32});
        roll_intGyro_MPU6050 = str2double(C{33});
        roll_comp_MPU6050 = str2double(C{34});
        roll_comp_latComp_MPU6050 = str2double(C{35});
        
        
        %% Print motor and Arduino data
        fprintf('[ID:%03d] Time:%f GoalPos:%03d  PresPos:%03d  GoalVel:%03d  PresVel:%03d\n\tax_MPU6050:%f ay_MPU6050:%f az_MPU6050:%f gx_MPU6050:%f gy_MPU6050:%f gz_MPU6050:%f\nroll_comp_MPU6050:%f roll_comp_latComp_MPU6050:%f roll_intGyro_MPU6050:%f\n\tax_PmodNAV:%f ay_PmodNAV:%f az_PmodNAV:%f gx_PmodNAV:%f gy_PmodNAV:%f gz_PmodNAV:%f\nroll_madgwick_PmodNAV:%f roll_mahony_PmodNAV:%f roll_comp_PmodNAV:%f roll_intGyro_PmodNAV:%f\n\n', DXL_ID, time(end), dxl_goal_position(index_pos), typecast(uint32(dxl_present_position), 'int32'), dxl_goal_velocity(index_vel), typecast(uint32(dxl_present_velocity), 'int32'), ax_MPU6050, ay_MPU6050, az_MPU6050, gx_MPU6050, gy_MPU6050, gz_MPU6050, roll_comp_MPU6050, roll_comp_latComp_MPU6050, roll_intGyro_MPU6050,ax_PmodNAV, ay_PmodNAV, az_PmodNAV, gx_PmodNAV, gy_PmodNAV, gz_PmodNAV, roll_madgwick_PmodNAV, roll_mahony_PmodNAV, roll_comp_PmodNAV, roll_intGyro_PmodNAV);
        
        
        %% Get current time
        time = [time toc(time_ini)];
        
        
        %% Plot motor and Arduino data
        % Add data to vectors
        dxl_present_position_vec = [dxl_present_position_vec dxl_present_position];
        dxl_present_velocity_vec = [dxl_present_velocity_vec dxl_present_velocity];
        
        roll_true_vec= [roll_true_vec -360*((dxl_present_position-DXL_MIDDLE_POSITION_VALUE)/4096)];
        
        % PmodNAV
        ax_vec_PmodNAV = [ax_vec_PmodNAV ax_PmodNAV];
        ay_vec_PmodNAV = [ay_vec_PmodNAV ay_PmodNAV];
        az_vec_PmodNAV = [az_vec_PmodNAV az_PmodNAV];
        gx_vec_PmodNAV = [gx_vec_PmodNAV gx_PmodNAV];
        gy_vec_PmodNAV = [gy_vec_PmodNAV gy_PmodNAV];
        gz_vec_PmodNAV = [gz_vec_PmodNAV gz_PmodNAV];
        mx_vec_PmodNAV = [mx_vec_PmodNAV mx_PmodNAV];
        my_vec_PmodNAV = [my_vec_PmodNAV my_PmodNAV];
        mz_vec_PmodNAV = [mz_vec_PmodNAV mz_PmodNAV];
        roll_madgwick_vec_PmodNAV = [roll_madgwick_vec_PmodNAV roll_madgwick_PmodNAV];
        roll_mahony_vec_PmodNAV = [roll_mahony_vec_PmodNAV roll_mahony_PmodNAV];
        roll_comp_vec_PmodNAV = [roll_comp_vec_PmodNAV roll_comp_PmodNAV];
        roll_intGyro_vec_PmodNAV = [roll_intGyro_vec_PmodNAV roll_intGyro_PmodNAV];
        
        % MPU6050
        ax_vec_MPU6050 = [ax_vec_MPU6050 ax_MPU6050];
        ay_vec_MPU6050 = [ay_vec_MPU6050 ay_MPU6050];
        az_vec_MPU6050 = [az_vec_MPU6050 az_MPU6050];
        gx_vec_MPU6050 = [gx_vec_MPU6050 gx_MPU6050];
        gy_vec_MPU6050 = [gy_vec_MPU6050 gy_MPU6050];
        gz_vec_MPU6050 = [gz_vec_MPU6050 gz_MPU6050];
        roll_comp_vec_MPU6050 = [roll_comp_vec_MPU6050 roll_comp_MPU6050];
        roll_comp_latComp_vec_MPU6050 = [roll_comp_latComp_vec_MPU6050 roll_comp_latComp_MPU6050];
        roll_intGyro_vec_MPU6050 = [roll_intGyro_vec_MPU6050 roll_intGyro_MPU6050];
        
        
        %plot
        if plot_roll
            refreshdata(h,'caller');
            drawnow;
        end
        
        if plot_acc_gyro
            refreshdata(hh,'caller');
            drawnow;
        end
        
        assignin('base','time',time);
        assignin('base','roll_true_vec',roll_true_vec);
        assignin('base','tline',tline);
        
        % PmodNAV
        assignin('base','ax_vec_PmodNAV',ax_vec_PmodNAV);
        assignin('base','ay_vec_PmodNAV',ay_vec_PmodNAV);
        assignin('base','az_vec_PmodNAV',az_vec_PmodNAV);
        assignin('base','gx_vec_PmodNAV',gx_vec_PmodNAV);
        assignin('base','gy_vec_PmodNAV',gy_vec_PmodNAV);
        assignin('base','gz_vec_PmodNAV',gz_vec_PmodNAV);
        assignin('base','mx_vec_PmodNAV',mx_vec_PmodNAV);
        assignin('base','my_vec_PmodNAV',my_vec_PmodNAV);
        assignin('base','mz_vec_PmodNAV',mz_vec_PmodNAV);
        assignin('base','roll_madgwick_vec_PmodNAV',roll_madgwick_vec_PmodNAV);
        assignin('base','roll_mahony_vec_PmodNAV',roll_mahony_vec_PmodNAV);
        assignin('base','roll_comp_vec_PmodNAV',roll_comp_vec_PmodNAV);
        assignin('base','roll_intGyro_vec_PmodNAV',roll_intGyro_vec_PmodNAV);
        
        % MPU6050
        assignin('base','ax_vec_MPU6050',ax_vec_MPU6050);
        assignin('base','ay_vec_MPU6050',ay_vec_MPU6050);
        assignin('base','az_vec_MPU6050',az_vec_MPU6050);
        assignin('base','gx_vec_MPU6050',gx_vec_MPU6050);
        assignin('base','gy_vec_MPU6050',gy_vec_MPU6050);
        assignin('base','gz_vec_MPU6050',gz_vec_MPU6050);
        assignin('base','roll_comp_vec_MPU6050',roll_comp_vec_MPU6050);
        assignin('base','roll_comp_latComp_vec_MPU6050',roll_comp_latComp_vec_MPU6050);
        assignin('base','roll_intGyro_vec_MPU6050',roll_intGyro_vec_MPU6050);

        
        %% Change the goal once it is reached
        if ~(abs(dxl_goal_position(index_pos) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            index_pos = mod(index_pos,length(dxl_goal_position))+1;   
            break;
        end
        
        if OPERATING_MODE == 1
            if ( (dxl_present_position > DXL_MAXIMUM_POSITION_VALUE && dxl_goal_velocity(index_vel) > 0) || (dxl_present_position < DXL_MINIMUM_POSITION_VALUE && dxl_goal_velocity(index_vel) < 0) )
    %         if ( (typecast(uint32(dxl_present_position), 'int32') > DXL_MAXIMUM_POSITION_VALUE && dxl_goal_velocity(index_vel) > 0) || (typecast(uint32(dxl_present_position), 'int32') < DXL_MINIMUM_POSITION_VALUE && dxl_goal_velocity(index_vel) < 0) )
                index_vel = mod(index_vel,length(dxl_goal_velocity))+1;
                break;
            end
        end
        
    end
end


% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

% Close Arduino port
fclose(serial_arduino);
end

%% Cleanup function
function myCleanupFunction(port_num, lib_name, PROTOCOL_VERSION,DXL_ID, ADDR_PRO_TORQUE_ENABLE, ADDR_PRO_OPERATING_MODE, ADDR_PRO_GOAL_POSITION, TORQUE_DISABLE, TORQUE_ENABLE, DXL_MIDDLE_POSITION_VALUE, COMM_SUCCESS, serial_arduino)
% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Choose position control operating mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_OPERATING_MODE, 3);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end
 
% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Write goal position to move bar back to the center position
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, typecast(int32(DXL_MIDDLE_POSITION_VALUE), 'uint32'));
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

pause(1);

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close portc
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

% Close Arduino port
fclose(serial_arduino);
end