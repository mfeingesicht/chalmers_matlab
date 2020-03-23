clear all
% close all
clc


%% Steering motor parameters
% Steering motor is a Maxon DCX 32 L 24V
R = 0.108;      % terminal resistance (Ohms)
L = 0.034*1e-3; % terminal inductance (H)
kt = 27.3*1e-3; % torque constant (Nm/A)
kb = 1/kt;      % speed constant (rad/s)/V
Jm = 72.8*1e-7; % Rotor inertia (kg.m2)
gr_sm = 111;    % Steering motor gear ratio


%% Belt parameters
gr_b = 1;       % Belt gear ratio
K = 1e6;        % Best stiffness (UNKNOWN)


%% Other parameters
N = gr_sm*gr_b;


%% Create cells to store data
amplitude_all = [];
omega_all = [];
delta_all = [];
P_all = [];

filename_all = {};
filename_all{end+1} = 'Kp416_Ti67/freq0.100000_amplitude0.174444.csv';
filename_all{end+1} = 'Kp416_Ti67/freq0.200000_amplitude0.174444.csv';
filename_all{end+1} = 'Kp416_Ti67/freq0.500000_amplitude0.174444.csv';
filename_all{end+1} = 'Kp416_Ti67/freq1.000000_amplitude0.174444.csv';
filename_all{end+1} = 'Kp416_Ti67/freq2.000000_amplitude0.174444.csv';
filename_all{end+1} = 'Kp416_Ti67/freq5.000000_amplitude0.017444.csv';
filename_all{end+1} = 'Kp416_Ti67/freq10.000000_amplitude0.017444.csv';

% filename_all{end+1} = 'Kp460_Ti1000/freq0.100000_amplitude0.174444.csv';
% filename_all{end+1} = 'Kp460_Ti1000/freq0.200000_amplitude0.174444.csv';
% filename_all{end+1} = 'Kp460_Ti1000/freq0.500000_amplitude0.174444.csv';
% filename_all{end+1} = 'Kp460_Ti1000/freq1.000000_amplitude0.174444.csv';
% filename_all{end+1} = 'Kp460_Ti1000/freq2.000000_amplitude0.174444.csv';
% filename_all{end+1} = 'Kp460_Ti1000/freq5.000000_amplitude0.087222.csv';
% filename_all{end+1} = 'Kp460_Ti1000/freq10.000000_amplitude0.087222.csv';

% Iterate over all files
for i=1:length(filename_all)
    filename = filename_all{i};
    %% Read data
    % Input : deltadot_ref -> Amplitude*cos(omega*t)
    % Output : delta (measured)
    data = csvread(filename,2);

    % Characteristics of the input sinusoidal signal
    t = regexp(filename,'\d+\.?\d*','match');
    amplitude = str2double(t{4});amplitude_all(i) = amplitude;
    omega = 2*pi*str2double(t{3});omega_all(i) = omega;

    % Steering motor controller
    Kp = str2double(t{1});       % Proportional gain of PI speed controller
    Ti = str2double(t{2})*1e-3;    % Integral time constant of PI speed controller


    time = data(:,1);
    delta = data(:,2);
    deltadot_ref = data(:,3);
    deltadot = data(:,4);

    %% Plot data
%     figure;plot(time,deltadot_ref);title('Reference Steering Rate (Input)');
%     figure;plot(time,delta);title('Measured Steering Angle (Output)');


    %% Preprocess data
    % Remove start of dat to wait for motor to warm up
    if strcmp(filename,'Kp416_Ti67/freq10.000000_amplitude0.017444.csv')
        cut_start = find(time>0.5,1);
    else
        cut_start = find(time>1,1);
    end
    T = time(cut_start:end);
    U = deltadot_ref(cut_start:end);
    Y = delta(cut_start:end);


    %% Identify model
    % u = a_u + b_u*t + c_u*sin(omega*t) + d_u*cos(omega*t)
    % y = a_y + b_y*t + c_y*sin(omega*t) + d_y*cos(omega*t)

    % Least-square identification
    % Phi = [1 t sin(omega*t) cos(omega*t)]
    % X   = [a b      c             d     ]'
    % u   = Phi*Xu
    % y   = Phi*Xy
    Phi = [ones(length(T),1) T sin(omega*T) cos(omega*T)];
    Xy = Phi\Y;
    Xu = Phi\U;

    % Plot data vs identified model
%     figure;hold on;plot(T,Y);plot(T,Phi*Xy);title('Reference Steering Rate (Input)');legend('Y','Phi*Xy');
%     figure;hold on;plot(T,U);plot(T,Phi*Xu);title('Measured Steering Angle (Output)');legend('U','Phi*Xu');

    meanY = 0;
    meanU = 0;
    A_data = 0;
    Asindelta_data = 0;
    B_data = 0;
    Bsindelta_data = 0;
    Asindelta_detrended = 0;
    Bsindelta_detrended = 0;
    switch filename
        case 'Kp416_Ti67/freq00.100000_amplitude0.174444.csv'
            meanY = (0.1193+(-0.2143))/2;
            meanU = 0;

            A_data = 0.1096;
            Asindelta_data = 0.1095;
            B_data = 0.1193;
            Bsindelta_data = 0.1193;

            Asindelta_detrended = 0.1096;
            Bsindelta_detrended = 0.1741;
        case 'Kp416_Ti67/freq00.200000_amplitude0.174444.csv'
            meanY = (0.1355+(-0.2066))/2;
            meanU = 0;

            A_data = 0.2192;
            Asindelta_data = 0.2192;
            B_data = 0.1357;
            Bsindelta_data = 0.1355;

            Asindelta_detrended = 0.2192;
            Bsindelta_detrended = 0.1751;
        case 'Kp416_Ti67/freq00.500000_amplitude0.174444.csv'
            meanY = (0.1989+(-0.1475))/2;
            meanU = 0;

            A_data = 0.548;
            Asindelta_data = 0.5465;
            B_data = 0.1989;
            Bsindelta_data = 0.1989;

            Asindelta_detrended = 0.5474;
            Bsindelta_detrended = 0.1755;
        case 'Kp416_Ti67/freq1.000000_amplitude0.174444.csv'
            meanY = (0.1903+(-0.1412))/2;
            meanU = 0;

            A_data = 1.096;
            Asindelta_data = 1.091;
            B_data = 0.1903;
            Bsindelta_data = 0.1896;

            Asindelta_detrended = 1.091;
            Bsindelta_detrended = 0.1647;
        case 'Kp416_Ti67/freq2.000000_amplitude0.174444.csv'
            meanY = (0.014+(-0.178))/2;
            meanU = 0;

            A_data = 2.191;
            Asindelta_data = 2.1640;
            B_data = 0.01939;
            Bsindelta_data = 0.014;

            Asindelta_detrended = 2.162;
            Bsindelta_detrended = 0.092;
        case 'Kp416_Ti67/freq5.000000_amplitude0.017444.csv'
            meanY = (0.02134+(-0.0142))/2;
            meanU = 0;

            A_data = 0.548;
            Asindelta_data = 0.5080;
            B_data = 0.02182;
            Bsindelta_data = 0.02134;

            Asindelta_detrended = 0.5155;
            Bsindelta_detrended = 0.0175;
        case 'Kp416_Ti67/freq10.000000_amplitude0.017444.csv'
            meanY = (0.013965+(-0.007))/2;
            meanU = 0;

            A_data = 1.096;
            Asindelta_data = 0.9285;
            B_data = 0.01664;
            Bsindelta_data = 0.013965;

            Asindelta_detrended = 0.9393;
            Bsindelta_detrended = 0.01150;
    end
    % Lissajou figure of data
%     figure;plot(U,Y);grid on;hold on;title('Lissajou figure');
%     xline(meanU);yline(meanY);

    P_data = (B_data-meanY)/(A_data-meanU);
    delta_A_data = -(90+90-rad2deg(asin(Asindelta_data/A_data)));
    delta_B_data = -90-rad2deg(asin(Bsindelta_data/B_data));

    % Detrended Lissajou figure (of dentrended identified model : a = 0, b = 0)
%     figure;plot(Phi(:,end-1:end)*Xu(end-1:end),Phi(:,end-1:end)*Xy(end-1:end));grid on;title('Detrended Lissajou figure');
    A_detrended = max(Phi(:,end-1:end)*Xu(end-1:end));
    B_detrended = max(Phi(:,end-1:end)*Xy(end-1:end));
    P_detrended = B_detrended/A_detrended;
    delta_A_detrended = rad2deg(asin(Asindelta_detrended/A_detrended));delta_A_detrended = -(90+90-delta_A_detrended);
    delta_B_detrended = rad2deg(asin(Bsindelta_detrended/B_detrended));delta_B_detrended = -(90+90-delta_B_detrended);

    % U = c_u sin(wt) + d_u cos(wt) = A sin(wt + delta_u)
    delta_u = rad2deg(atan2(Xu(end),Xu(end-1)));
    A = sqrt(Xu(end)^2+Xu(end-1)^2);

    % Y = c_y sin(wt) + d_y cos(wt) = B sin(wt + delta_y)
    delta_y = rad2deg(atan2(Xy(end),Xy(end-1)));
    B = sqrt(Xy(end)^2+Xy(end-1)^2);

    delta = delta_y - delta_u;delta_all(i) = delta;
    P = B/A;P_all(i) = P;

    % Make Bode+Nyquist+Nichols plot with mag2db(P) as gain and delta as phase
    % Then plot transfer function of steering motor and iterate over J to find
    % the best fit
end


%% Bode plot from data
% figure;hold on;title('Bode plot');
% subplot(211);semilogx(omega_all/(2*pi),mag2db(P_all),'*-');ylabel('Gain (dB)');
% subplot(212);semilogx(omega_all/(2*pi),delta_all,'*-');ylabel('Phase (deg)');xlabel('Frequency (Hz)');


%% Transfer function of the system
Kp = 1e10;
Ti = 1e10;

Jf = 0.2553; % Front fork inertia (kg.m2)
J = Jm + Jf/N^2;
s = tf('s');
tf_PI = Kp * (1 + 1/(Ti*s));
tf_V2deltadot = kt / ( (R+L*s)*J*s + kt^2);
% tf_deltadotref2delta = (tf_PI*tf_V2deltadot / (1+tf_PI*tf_V2deltadot))*(1/s) * (1/N);
tf_deltadotref2delta = (tf_PI*tf_V2deltadot / (1+tf_PI*tf_V2deltadot))*(1/s);
tf_deltadotref2delta = minreal(tf_deltadotref2delta);
tf_deltadotref2delta_noTi = (Kp*kt / (L*J*s^2 + R*J*s + (kt^2+Kp*kt))) * (1/s) * (1/N);

h = figure;hold on;bodeplot(tf_deltadotref2delta,omega_all);
[P_tf_all,delta_tf_all] = bode(tf_deltadotref2delta,omega_all);
P_tf_all = squeeze(P_tf_all);
delta_tf_all = squeeze(delta_tf_all);

children = get(h, 'Children') % use this handle to obtain list of figure's children
% We see that children has 3 objects:
% 1) Context Menu 2) Axis object to Phase Plot 3) Axis object to Magnitude Plot
magChild = children(3);          % Pick a handle to axes of magnitude in bode diagram
axes(magChild)                   % Make those axes current
semilogx(omega_all,mag2db(P_all),'r*');
phaseChild = children(2);          % Pick a handle to axes of phase in bode diagram
axes(phaseChild)                   % Make those axes current
semilogx(omega_all,delta_all,'r*');ylim('auto');ylim([-270 -90]);
legend('Model','Data')

[P_all' P_tf_all],
[delta_all' delta_tf_all],