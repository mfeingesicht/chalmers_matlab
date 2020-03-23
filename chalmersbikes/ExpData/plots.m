close all
clear all
clc


%% Extract data
% Choice of data file
% T = readtable('BikeData-20200313-103548.csv');
% T = readtable('BikeData-20200313-153029.csv');
T = readtable('BikeData-20200313-161318.csv');

time = T.Time;
phi = T.Phi;
delta = T.Delta;
phidot = T.PhiDot;


%% Plots
figure;hold on;plot(time,phi);plot(time,delta);plot(time,phidot);plot(time,cumtrapz(time,phidot));plot(time,time*mean(phidot))
legend('Phi','Delta','PhiDot','Integral of mean(PhiDot)','Integral of PhiDot');


%% FFT
Fs = 100;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = length(phi);             % Length of signal
t = (0:L-1)*T;        % Time vector
f = Fs*(0:(L/2))/L;

X_phi = phi;
Y_phi = fft(X_phi);
P2_phi = abs(Y_phi/L);
P1_phi = P2_phi(1:L/2+1);
P1_phi(2:end-1) = 2*P1_phi(2:end-1);

X_delta = delta;
Y_delta = fft(X_delta);
P2_delta = abs(Y_delta/L);
P1_delta = P2_delta(1:L/2+1);
P1_delta(2:end-1) = 2*P1_delta(2:end-1);

figure;hold on;plot(f,P1_phi);plot(f,P1_delta);
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
legend('Phi','Delta');



%% 
% figure;hold on;plot(T.Time(2:end),diff(T.Time));xlabel('Time (s)');ylabel('Sampling time (s)'); title('Sampling time on the black bike');
% figure;hold on;plot(T.Time,T.Phi*180/pi);plot(T.Time,T.Delta*180/pi);title('Phi and Delta');xlabel('Time (s)');ylabel('Phi and Delta (deg)');legend('Phi','Delta');
% figure;hold on;plot(T.Time,T.PhiDot*180/pi);title('PhiDot');xlabel('Time (s)');ylabel('PhiDot (deg/s)');
% figure;hold on;plot(T.Time,kp*T.PhiDot*180/pi);plot(T.Time,ki*cumtrapz(T.Time,T.PhiDot*180/pi));plot(T.Time(2:end),kd*diff(T.PhiDot*180/pi)./diff(T.Time));xlabel('Time (s)');legend('kp*PhiDot','ki*PhiDot/s','kd*s*PhiDot');
% figure;hold on;plot(T.Time,[0 ; diff(T.Delta*180/pi)./diff(T.Time)]);plot(T.Time,(kp*T.PhiDot*180/pi)+(ki*cumtrapz(T.Time,T.PhiDot*180/pi))+[0 ; (kd*diff(T.PhiDot*180/pi)./diff(T.Time))]);xlabel('Time (s)');legend('DeltaDot','Controlinput');


% deltadot = [0 ; diff(T.Delta*180/pi)./diff(T.Time)];
% controlinput = (kp*T.PhiDot*180/pi)+(ki*cumtrapz(T.Time,T.PhiDot*180/pi))+[0 ; (kd*diff(T.PhiDot*180/pi)./diff(T.Time))];
% [C,lags] = xcorr(deltadot,controlinput);
% figure;plot(lags,C)