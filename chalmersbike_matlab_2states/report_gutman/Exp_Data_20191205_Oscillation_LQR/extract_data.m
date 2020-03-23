% clear all
% close all
% clc

%% Extract data
% Bike parameters
report_gutman;

% Read data into a table
% Data_exp = readtable('BikeData-20191205-104721.csv');
% Data_exp = readtable('BikeData-20191205-110353.csv');
% Data_exp = readtable('BikeData-20191205-110646.csv');
% Data_exp = readtable('BikeData-20191205-111910.csv');
Data_exp = readtable('BikeData-20191205-112027.csv');
% Data_exp = readtable('BikeData-20191205-113219.csv');

Data_exp = rmmissing(Data_exp);

% Extract from table of data
time = Data_exp.Time;
phi = rad2deg(Data_exp.Phi);
phidot = rad2deg(Data_exp.PhiDot);
delta = rad2deg(Data_exp.Delta);
deltadot_ref = rad2deg(Data_exp.ControlInput);
delta_ref = rad2deg(Data_exp.delta_ctrl_ref);

% Simulation
sim('model_report_gutman.slx');


%% Plots
figure;hold on;plot(time,delta_ref);plot(time,delta);plot(realised_input);
title('Steering Angle');
xlabel('Time (s)');
ylabel('Steering Angle (deg)');
legend('Reference Steering Angle','Steering Angle on the Real Bike','Steering Angle in Simulation');

figure;hold on;plot(time(2:end),deltadot_ref(2:end));plot(time(2:end),diff(delta)/0.01);plot(realized_steeringrate);
title('Steering Rate');
xlabel('Time (s)');
ylabel('Steering Rate (deg)');
legend('Reference Steering Rate','Steering Rate on the Real Bike','Steering Rate in Simulation');

[c_bike_delta,lags_bike_delta] = xcorr(delta_ref,delta);
[c_sim_delta,lags_sim_delta] = xcorr(calculated_input.Data,realised_input.Data);
figure;plot(lags_bike_delta,c_bike_delta);title('xcorr steering angle bike');
figure;plot(lags_sim_delta,c_sim_delta);title('xcorr steering angle sim');

% [c_bike_deltadot,lags_bike_deltadot] = xcorr(deltadot_ref(2:end),diff(delta)/0.01);
[c_bike_deltadot,lags_bike_deltadot] = xcorr(deltadot_ref(1:end-1),diff(delta)/0.01);
[c_sim_deltadot,lags_sim_deltadot] = xcorr(calculated_steeringrate.Data,realized_steeringrate.Data);
figure;plot(lags_bike_deltadot,c_bike_deltadot);title('xcorr steering rate bike');
figure;plot(lags_sim_deltadot,c_sim_deltadot);title('xcorr steering rate sim');


fprintf('Delay on delta on the bike : %d\n',lags_bike_delta(c_bike_delta==max(c_bike_delta)));
fprintf('Delay on delta in simulation : %d\n',lags_sim_delta(c_sim_delta==max(c_sim_delta)));
fprintf('Delay on deltadot on the bike : %d\n',lags_bike_deltadot(c_bike_deltadot==max(c_bike_deltadot)));
fprintf('Delay on deltadot in simulation : %d\n',lags_sim_deltadot(c_sim_deltadot==max(c_sim_deltadot)));