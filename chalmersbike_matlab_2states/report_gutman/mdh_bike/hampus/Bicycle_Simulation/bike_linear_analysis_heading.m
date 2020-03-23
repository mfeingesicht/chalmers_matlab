%% Load simulation parameters
main;


%% Vector of speeds at which the anlysis should be run
P_heading = -0.7; % Direction control P gain
kp_vec = [-0.7 -0.4 -0.1 0.3 0.7];
kp_vec = [-0.2];
% kp_vec = [-0.001 -0.01 -0.05 -0.1];
kd_vec = [0 0.0001 0.001 0.01 0.05 0.1 1];


%% Create pole-zero map, bole plot, nyquist plot, nichols plot
% Open Simulink file
simulink_file = 'model';
open_system(simulink_file);

% Select input and output for linearization
io(1) = linio([simulink_file '/Reference Heading'],1,'openinput');
io(2) = linio([simulink_file '/Global Angles and Coordinates Calculator - Position'],2,'openoutput');

% Create figures
fpz = figure;hold on;
fbo = figure;hold on;
fny = figure;hold on;
fni = figure;hold on;

linsyscell = cell(length(kp_vec)*length(kd_vec),1);
lgd = cell(length(kp_vec)*length(kd_vec),1);
for i=1:length(kp_vec)
    for j = 1:length(kd_vec)
        P_heading = kp_vec(i);
        D_heading = kd_vec(j);

        % Legend
        lgd{(i-1)*length(kd_vec)+j} = ['Kp = ' , num2str(P_heading) , ' ; Kd = ', num2str(D_heading)];

        % Compute state-space matrice for current speed
        num = [(inertia_front*b)/(m*h) a*v v^2];
        den = [b*h 0 -b*g];
        [A,B,C,D] = tf2ss(num,den);

        % Linearize Simulink model
        linsyscell{(i-1)*length(kd_vec)+j} = linearize(simulink_file,io);

        % Pole-Zero Map
        figure(fpz);
        hpz = pzplot(linsyscell{(i-1)*length(kd_vec)+j});
        set(hpz.allaxes.Children(1).Children, 'MarkerSize', 16)


        % Bode plot
        figure(fbo);
        hb = bodeplot(linsyscell{(i-1)*length(kd_vec)+j});
        setoptions(hb,'FreqUnits','Hz');


        % Nyquist plot
        figure(fny);
        hny = nyquistplot(linsyscell{(i-1)*length(kd_vec)+j});


        % Nichols plot
        figure(fni);
        hni = nicholsplot(linsyscell{(i-1)*length(kd_vec)+j});
    end
end
fbo.Children(3).Title.String = 'From: Reference Heading  To: Measured Heading';
fny.Children(2).Title.String = 'From: Reference Heading  To: Measured Heading';
fni.Children(2).Title.String = 'From: Reference Heading  To: Measured Heading';
figure(fpz);legend(lgd)
figure(fbo);legend(lgd)
figure(fny);legend(lgd)
figure(fni);legend(lgd)