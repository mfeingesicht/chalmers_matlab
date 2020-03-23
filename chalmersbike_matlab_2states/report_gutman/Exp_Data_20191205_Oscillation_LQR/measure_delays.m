% clear all
% close all
% clc

%% Extract data
% Read data into a table
% Data_exp = readtable('BikeData-20191205-104721.csv');
% Data_exp = readtable('BikeData-20191205-110353.csv');
% Data_exp = readtable('BikeData-20191205-110646.csv');
Data_exp = readtable('BikeData-20191205-111910.csv');
Data_exp = readtable('BikeData-20191205-112027.csv');
Data_exp = readtable('BikeData-20191205-113219.csv');

% Extract from table of data
time = Data_exp.Time;
phi = rad2deg(Data_exp.Phi);
phidot = rad2deg(Data_exp.PhiDot);
delta = rad2deg(Data_exp.Delta);
deltadot_ref = rad2deg(Data_exp.ControlInput);
delta_ref = rad2deg(Data_exp.delta_ctrl_ref);


%% Plots
[c_bike_delta,lags_bike_delta] = xcorr(delta_ref,delta);
figure;plot(lags_bike_delta,c_bike_delta);title('xcorr steering angle bike');

[c_bike_deltadot,lags_bike_deltadot] = xcorr(deltadot_ref(2:end),diff(delta)/0.01);
figure;plot(lags_bike_deltadot,c_bike_deltadot);title('xcorr steering rate bike');

sprintf('Delay on delta : %d',lags_bike_delta(c_bike_delta==max(c_bike_delta)));
sprintf('Delay on deltadot : %d',lags_bike_deltadot(c_bike_deltadot==max(c_bike_deltadot)));