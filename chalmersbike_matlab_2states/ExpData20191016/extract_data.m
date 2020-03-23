% clear all
% close all
% clc


% Data_exp = readtable('BikeData-20191016-100201.csv');
Data_exp = readtable('BikeData-20191016-094153.csv');
% Data_exp = readtable('BikeData-20191016-094404.csv');
% Data_exp = readtable('BikeData-20191016-094715.csv');
% Data_exp = readtable('BikeData-20191016-095429.csv');
% Data_exp = readtable('BikeData-20191016-095924.csv');
% Data_exp = readtable('BikeData-20191016-100342.csv');
% Data_exp = readtable('BikeData-20191016-100555.csv');
% Data_exp = readtable('BikeData-20191016-100731.csv');


Ts = 0.01;

time = Data_exp.Time;
phi = rad2deg(Data_exp.Phi);
phidot = rad2deg(Data_exp.PhiDot);
delta = rad2deg(Data_exp.Delta);
deltadot_ref = rad2deg(Data_exp.ControlInput);
delta_ref = rad2deg(Data_exp.delta_ctrl_ref);

figure;hold on;plot(time,delta_ref);plot(time,delta);plot(time,cumtrapz(time,deltadot_ref));
title('Steering Angle');
xlabel('Time (s)');
ylabel('Steering Angle (deg)');
legend('Reference Steering Angle','Steering Angle','Integrated Reference Steering Rate');