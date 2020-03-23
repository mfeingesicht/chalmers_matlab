% clear all
% close all
% clc

% t = readtable('20200221-090356-SensorTest_Lukas_SteeringMotor.csv');
t = readtable('20200221-104414-SensorTest_Lukas_SteeringMotor.csv');

delta = t.tdelta_deg_;
deltadot_ref = t.AngularVelocity_deg_s_;
t = t.Time_s_;
deltadot = [0 ; diff(delta)./diff(t)];

figure;hold on;plot(t,deltadot_ref);plot(t,deltadot);
figure;hold on;plot(t,deltadot_ref/50);plot(t,deltadot/50);