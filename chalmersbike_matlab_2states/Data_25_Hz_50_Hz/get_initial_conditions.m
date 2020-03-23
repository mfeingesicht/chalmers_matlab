clear all
close all
clc


% Data_exp = readtable('BikeData-20191011-185547.csv');
Data_exp = readtable('BikeData-20191011-185946.csv');

Ts = 0.025;

time = Data_exp.Time;
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

bandwidth_window = (60:87);
bandwidth_window = (12:25);
Y_filtered = Y;Y_filtered(bandwidth_window) = Y_filtered(bandwidth_window)/1000;
P2_filtered = abs(Y_filtered/L);
P1_filtered = P2_filtered(1:L/2+1);
P1_filtered(2:end-1) = 2*P1_filtered(2:end-1);
% figure;plot(f,P1);hold on;plot(f,P1_filtered);


% X_filtered = ifft(Y_filtered);
X_filtered = ifft(Y_filtered,'symmetric');


% % % [X_bandstop,D_bandstop] = bandstop(X,[1.51 2.21],Fs);
% [b,a]=butter(3,[1.51 2.21]/(Fs/2),'stop');
% [b,a]=butter(1,[1 3]/(Fs/2),'stop');
[b,a]=butter(1,[1.5 2.5]/(Fs/2),'stop');
[mag_butter,freq_butter] = freqz(b,a,2048,Fs);
figure;plot(freq_butter,mag_butter);
% fvtool(b,a);
X_bandstop = filter(b,a,X);

% figure;plot(X);hold on;plot(X_filtered);
figure;plot(X);hold on;plot(real(X_filtered));plot(X_bandstop);legend('X','X\_filt','bandstop');