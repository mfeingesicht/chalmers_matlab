clear all
close all
clc


%% Steering motor
Ts = 0.01;

num_tf_steering = [253.4];
den_tf_steering = [1 245.9 0];
sys_tf_steering = tf(num_tf_steering,den_tf_steering);

sys_tf_steering_dis = c2d(sys_tf_steering,Ts);

P_steering_position = 30;
I_steering_position = 0;
D_steering_position = 0;
K_feedforward_steering = 0;


%% Experimental data
% Data_exp = readtable('strAngCon3030.csv');
Data_exp = readtable('StterAngleCtrl20191105.csv');

time = Data_exp.Time;
delta = rad2deg(Data_exp.RealizedDelta);
delta_ref = rad2deg(Data_exp.DeltaRef);
deltadot = rad2deg(Data_exp.DeltaDot);
deltadot_ref = rad2deg(Data_exp.DeltaDotRef);


%% Simulation
sim('steering_validation');


%% Plots
figure;hold on;plot(time,delta);plot(simout_delta);plot(time,delta_ref);title('Steering Angle');xlabel('Time (s)');ylabel('Steering Angle (deg)');legend('Experimental Steering Angle','Simulation Steering Angle','Reference Steering Angle');xlim([0 10]);
% figure;hold on;plot(time,deltadot);plot(simout_deltadot);title('Steering Rate');xlabel('Time (s)');ylabel('Steering Rate (deg/s)');legend('Experimental Steering Rate','Simulation Steering Rate');ylim([-1 5]);xlim([0 10]);
% figure;hold on;plot(time,deltadot);plot(simout_deltadot);title('Steering Rate');xlabel('Time (s)');ylabel('Steering Rate (deg/s)');legend('Experimental Steering Rate','Simulation Steering Rate');ylim([-1 5]);xlim([0 10]);xlim([0 0.5]);ylim('auto')