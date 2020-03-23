clear all
close all
clc

% data = readtable('EncoderlargeSteps.csv');
data = readtable('EncoderTest01.csv');
% data = readtable('EncoderTest02.csv');

Ts = 0.04;

ignored_window = 10;
time = data.Time(ignored_window:end)-data.Time(ignored_window);
delta = data.delta(ignored_window:end); % Measured steering angle by encoder
deltadot_ref = data.angVelref(ignored_window:end); % Reference steering rate
deltadot = data.Realized(ignored_window:end); % Derivative of encoder output

figure;stairs(time,deltadot_ref);hold on;plot(time,deltadot);

% tfestimate(deltadot_ref,deltadot);

data_iddata = iddata(deltadot,deltadot_ref,Ts);

% sys = {};
% for np = 1:5
% %     sys = tfest(iddata(deltadot,deltadot_ref,Ts),np)
% %     sys{np} = tfest(iddata(deltadot,deltadot_ref,Ts),np,[],NaN);
%     sys{np} = tfest(data_iddata,np,[],0.04);
%     sys{np},
%     figure;bode(sys{np});title(['Sys ',num2str(np)])
%     figure;lsim(sys{np},deltadot_ref,0.04*(0:(length(time)-1)));hold on;plot(time,deltadot,'r');hold on;title(['Sys ',num2str(np)]);legend('Step response of identified transfer function','Measured Steering Rate');
% end