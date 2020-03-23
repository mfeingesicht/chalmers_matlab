%% Load simulation parameters
main;


%% Vector of speeds at which the anlysis should be run
kd_vec = [0 0.001 0.01 0.1 1 10];


%% Create pole-zero map, bole plot, nyquist plot, nichols plot
% Open Simulink file
simulink_file = 'model';
open_system(simulink_file);

% Select input and output for linearization
io(1) = linio([simulink_file '/Reference Position'],2,'openinput');
io(2) = linio([simulink_file '/Global Angles and Coordinates Calculator - Position'],2,'openoutput');

% Create figures
fpz = figure;hold on;
fbo = figure;hold on;
fny = figure;hold on;
fni = figure;hold on;

linsyscell = cell(4,1);
lgd = cell(4,1);
for i = 1:length(kd_vec)
    D_direct = kd_vec(i);

    % Legend
    lgd{i} = ['Kd = ', num2str(D_direct)];

    % Compute state-space matrice for current speed
    num = [(inertia_front*b)/(m*h) a*v v^2];
    den = [b*h 0 -b*g];
    [A,B,C,D] = tf2ss(num,den);
    
    % Linearize Simulink model
    linsyscell{i} = linearize(simulink_file,io);
    
    % 
    nichols(linsyscell{i})
    
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
fbo.Children(3).Title.String = 'From: Reference Heading  To: Measured Heading';
fny.Children(2).Title.String = 'From: Reference Heading  To: Measured Heading';
fni.Children(2).Title.String = 'From: Reference Heading  To: Measured Heading';
figure(fpz);legend(lgd)
figure(fbo);legend(lgd)
figure(fny);legend(lgd)
figure(fni);legend(lgd)