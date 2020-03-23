clear all
close all
clc


%% Load model
load('bike_model.mat');


%% Pole-Zero Map
figure;
hpz = pzplot(bike_model);
set(hpz.allaxes.Children(1).Children, 'MarkerSize', 16)


%% Bode plot
figure;
hb = bodeplot(bike_model);
setoptions(hb,'FreqUnits','Hz');
fb = gcf;
fb.Children(3).Title.String = 'From: Steering Rate  To: Roll Rate';


%% Nyquist plot
figure;
hny = nyquistplot(bike_model);
fny = gcf;
fny.Children(2).Title.String = 'From: Steering Rate  To: Roll Rate';


%% Nichols plot
figure;
hni = nicholsplot(bike_model);
fni = gcf;
fni.Children(2).Title.String = 'From: Steering Rate  To: Roll Rate';