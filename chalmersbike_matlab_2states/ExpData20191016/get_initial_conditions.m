clear all
close all
clc


% Data_exp = readtable('BikeData-20191016-100201.csv');
% Data_exp = readtable('BikeData-20191016-094153.csv');
% Data_exp = readtable('BikeData-20191016-094404.csv');
% Data_exp = readtable('BikeData-20191016-094715.csv');
% Data_exp = readtable('BikeData-20191016-095429.csv');
% Data_exp = readtable('BikeData-20191016-095924.csv');
% Data_exp = readtable('BikeData-20191016-100342.csv');
% Data_exp = readtable('BikeData-20191016-100555.csv');
Data_exp = readtable('BikeData-20191016-100731.csv');

Ts = 0.02;

time = Data_exp.Time;
calc_time = Data_exp.CalculationTime;
phi = Data_exp.Phi;
delta = Data_exp.Delta;
phidot = Data_exp.PhiDot;
deltadot = Data_exp.ControlInput;
x1_ref = Data_exp.x1_ref;
x2_ref = Data_exp.x2_ref;
x1 = Data_exp.x1;
x2 = Data_exp.x2;
delta_ref = Data_exp.delta_k;

figure;plot(delta);title('delta');


% X = delta;
X = deltadot;
Fs = 1/Ts;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = length(time);     % Length of signal
Y = fft(X);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;
figure;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

% % % % [X_bandstop,D_bandstop] = bandstop(X,[1.51 2.21],Fs);
% % % [b,a]=butter(3,[1.51 2.21]/(Fs/2),'stop');
% % % [b,a]=butter(1,[1 3]/(Fs/2),'stop');
% [b,a]=butter(2,[1.5 3.5]/(Fs/2),'stop');
[b,a]=butter(2,8/(Fs/2),'low');
[mag_butter,freq_butter] = freqz(b,a,2048,Fs);
figure;semilogx(freq_butter,20*log10(abs(mag_butter)));
% fvtool(b,a);
X_bandstop = filter(b,a,X);

% figure;plot(X);hold on;plot(X_filtered);
figure;plot(X);hold on;plot(X_bandstop);legend('X','bandstop');