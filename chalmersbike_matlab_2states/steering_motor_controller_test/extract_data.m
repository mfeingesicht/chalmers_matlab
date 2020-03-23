% clear all
% close all
% clc

% Data_exp = readtable('csv_test_steering_Kp0_Ki0_D1.csv');
% Data_exp = readtable('csv_test_steering_Kp5_Ki0_D0.csv');
% Data_exp = readtable('csv_test_steering_Kp5_Ki0_D1.csv');
Data_exp = readtable('csv_test_steering_Kp5_Ki1_D0.csv');
% Data_exp = readtable('csv_test_steering_Kp5_Ki1_Kd02.csv');
% Data_exp = readtable('csv_test_steering_Kp5_Ki1_Kd1.csv');

Ts = 0.01;

time = Data_exp.Time;
delta = rad2deg(Data_exp.delta);
delta_ref = rad2deg(Data_exp.AngRef);
deltadot = rad2deg(Data_exp.Realized);
deltadot_ref = rad2deg(Data_exp.angVelref);

figure;hold on;plot(time,delta);plot(simout_delta);plot(time,delta_ref);title('Steering Angle');xlabel('Time (s)');ylabel('Steering Angle (deg)');legend('Experimental Steering Angle','Simulation Steering Angle','Reference Steering Angle');xlim([0 10]);
figure;hold on;plot(time,deltadot);plot(simout_deltadot);title('Steering Rate');xlabel('Time (s)');ylabel('Steering Rate (deg/s)');legend('Experimental Steering Rate','Simulation Steering Rate');ylim([-1 5]);xlim([0 10]);
figure;hold on;plot(time,deltadot);plot(simout_deltadot);title('Steering Rate');xlabel('Time (s)');ylabel('Steering Rate (deg/s)');legend('Experimental Steering Rate','Simulation Steering Rate');ylim([-1 5]);xlim([0 10]);xlim([0 0.5]);ylim('auto')


%% Step
figure;hold on;plot(simout_delta_ref);plot(simout_delta);title('Step response');legend('Steering Angle Step Reference','Steering Angle Step Response');xlabel('Time (s)');ylabel('Steering Angle (deg)');