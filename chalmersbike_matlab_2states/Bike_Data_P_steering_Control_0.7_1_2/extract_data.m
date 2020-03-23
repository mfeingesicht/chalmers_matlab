clear all
% close all
clc

Data_exp = readtable('BikeData-20191115-110036.csv');
% Data_exp = readtable('BikeData-20191115-110309.csv');
% Data_exp = readtable('BikeData-20191115-111150.csv');

Ts = 0.01;

time = Data_exp.Time;
phi = rad2deg(Data_exp.Phi);
delta = rad2deg(Data_exp.Delta);
phidot = rad2deg(Data_exp.PhiDot);
% delta_ref = rad2deg(Data_exp.AngRef);
% deltadot = rad2deg(Data_exp.Realized);
% deltadot_ref = rad2deg(Data_exp.angVelref);

figure;hold on;plot(phi);plot(delta);plot(phidot);legend('Phi','Delta','PhiDot');
% figure;hold on;plot(time,delta);plot(simout_delta);plot(time,delta_ref);title('Steering Angle');xlabel('Time (s)');ylabel('Steering Angle (deg)');legend('Experimental Steering Angle','Simulation Steering Angle','Reference Steering Angle');xlim([0 10]);
% figure;hold on;plot(time,deltadot);plot(simout_deltadot);title('Steering Rate');xlabel('Time (s)');ylabel('Steering Rate (deg/s)');legend('Experimental Steering Rate','Simulation Steering Rate');ylim([-1 5]);xlim([0 10]);
% figure;hold on;plot(time,deltadot);plot(simout_deltadot);title('Steering Rate');xlabel('Time (s)');ylabel('Steering Rate (deg/s)');legend('Experimental Steering Rate','Simulation Steering Rate');ylim([-1 5]);xlim([0 10]);xlim([0 0.5]);ylim('auto')

%% FFT phidot
Fs = 1/Ts;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       

X = phidot;
Y = fft(X);
L = length(Y);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;
figure;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')