%% Load simulation parameters
report_gutman;
v = 4;


%% Vector of Kd at which the anlysis should be run
kp = 1;
kd = 0.1;

ki_vec = [0 0.01 0.1 0.5 1];


%% Create pole-zero map, bole plot, nyquist plot, nichols plot
% Open Simulink file
simulink_file = 'bike_pd_steering';
open_system(simulink_file);

% Select input and output for linearization
io(1) = linio([simulink_file '/Roll Rate Reference'],1,'openinput');
io(2) = linio([simulink_file '/Physical Bike'],7,'output');

% Create figures
fpz = figure;hold on;
fbo = figure;hold on;
fny = figure;hold on;
fni = figure;hold on;

linsyscell = cell(length(ki_vec),1);
lgd = cell(length(ki_vec),1);
for i = 1:length(ki_vec)
    ki = ki_vec(i);

    % Compute state-space matrice for current speed
    num = [(inertia_front*b)/(m*h) a*v v^2];
    den = [b*h 0 -b*g];
    [A,B,C,D] = tf2ss(num,den);

    % Add speed to legend
    lgd{i} = ['ki = ', num2str(ki)];
    
    % Linearize Simulink model
    linsyscell{i} = linearize(simulink_file,io);
    
    
    % Pole-Zero Map
    figure(fpz);
    hpz = pzplot(linsyscell{i});
    set(hpz.allaxes.Children(1).Children, 'MarkerSize', 16)


    % Bode plot
    figure(fbo);
    hb = bodeplot(linsyscell{i});
    setoptions(hb,'FreqUnits','Hz');


    % Nyquist plot
    figure(fny);
    hny = nyquistplot(linsyscell{i});


    % Nichols plot
    figure(fni);
    hni = nicholsplot(linsyscell{i});
end
fbo.Children(3).Title.String = 'From: Roll Rate Reference  To: Roll Rate';
fny.Children(2).Title.String = 'From: Roll Rate Reference  To: Roll Rate';
fni.Children(2).Title.String = 'From: Roll Rate Reference  To: Roll Rate';
figure(fpz);legend(lgd)
figure(fbo);legend(lgd)
figure(fny);legend(lgd)
figure(fni);legend(lgd)