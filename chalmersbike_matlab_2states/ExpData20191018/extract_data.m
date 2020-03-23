% clear all
% close all
% clc


Data_exp = readtable('BikeData-20191018-112818.csv');
% Data_exp = readtable('BikeData-20191018-112946.csv');
% Data_exp = readtable('BikeData-20191018-113406.csv');
% Data_exp = readtable('BikeData-20191018-113653.csv');
% Data_exp = readtable('BikeData-20191018-114030.csv');
% Data_exp = readtable('BikeData-20191018-115750.csv');


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