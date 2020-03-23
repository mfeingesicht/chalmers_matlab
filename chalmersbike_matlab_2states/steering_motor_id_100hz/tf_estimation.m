clear all
close all
clc

% data = readtable('strAngCon3030.csv');
% data = readtable('StterAngleCtrl20191105.csv');
data = readtable('20191023_100hz.csv');

Ts = 0.01;

ignored_window = 10;
time = data.Var1(ignored_window:end)-data.Var1(ignored_window);
delta = data.Var2(ignored_window:end); % Measured steering angle by encoder
deltadot_ref = data.Var3(ignored_window:end); % Reference steering rate
deltadot = data.Var4(ignored_window:end); % Derivative of encoder output

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

num = [262];
den = [1 254.4];

num = [253.4];
den = [1 245.9];

figure;hold on;plot(time(1364:1614)-time(1364),deltadot_ref(1364:1614));plot(time(1364:1614)-time(1364),(deltadot(1364:1614)+0.2)/2);