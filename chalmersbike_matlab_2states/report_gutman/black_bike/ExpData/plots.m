% close all
clear all
clc

Ts = 0.01;
kp = 1;
kd = 0.06;

T = readtable('BikeData-20200219-145039_Ki005.csv');ki = 0.05;
% T = readtable('BikeData-20200219-145841_Ki0.csv');ki = 0;

figure;hold on;plot(T.Time(2:end),diff(T.Time));xlabel('Time (s)');ylabel('Sampling time (s)'); title('Sampling time on the black bike');
figure;hold on;plot(T.Time,T.Phi*180/pi);plot(T.Time,T.Delta*180/pi);title('Phi and Delta');xlabel('Time (s)');ylabel('Phi and Delta (deg)');legend('Phi','Delta');
figure;hold on;plot(T.Time,T.PhiDot*180/pi);title('PhiDot');xlabel('Time (s)');ylabel('PhiDot (deg/s)');
figure;hold on;plot(T.Time,kp*T.PhiDot*180/pi);plot(T.Time,ki*cumtrapz(T.Time,T.PhiDot*180/pi));plot(T.Time(2:end),kd*diff(T.PhiDot*180/pi)./diff(T.Time));xlabel('Time (s)');legend('kp*PhiDot','ki*PhiDot/s','kd*s*PhiDot');
figure;hold on;plot(T.Time,[0 ; diff(T.Delta*180/pi)./diff(T.Time)]);plot(T.Time,(kp*T.PhiDot*180/pi)+(ki*cumtrapz(T.Time,T.PhiDot*180/pi))+[0 ; (kd*diff(T.PhiDot*180/pi)./diff(T.Time))]);xlabel('Time (s)');legend('DeltaDot','Controlinput');



deltadot = [0 ; diff(T.Delta*180/pi)./diff(T.Time)];
controlinput = (kp*T.PhiDot*180/pi)+(ki*cumtrapz(T.Time,T.PhiDot*180/pi))+[0 ; (kd*diff(T.PhiDot*180/pi)./diff(T.Time))];
[C,lags] = xcorr(deltadot,controlinput);
figure;plot(lags,C)