%% Load simulation parameters
report_gutman;


%% Vector of speeds at which the anlysis should be run
v_vec = 3:6;


%% Create pole-zero map, bole plot, nyquist plot, nichols plot
% Open Simulink file
simulink_file = 'bike';
open_system(simulink_file);

% Select input and output for linearization
io(1) = linio([simulink_file '/Steering Rate'],1,'openinput');
io(2) = linio([simulink_file '/Physical Bike'],7,'output');

% Create figures
fpz = figure;hold on;
fbo = figure;hold on;
fny = figure;hold on;
fni = figure;hold on;

linsyscell = cell(4,1);
lgd = cell(4,1);
for i = 1:length(v_vec)
    v = v_vec(i);

    % Compute state-space matrice for current speed
    num = [(inertia_front*b)/(m*h) a*v v^2];
    den = [b*h 0 -b*g];
    [A,B,C,D] = tf2ss(num,den);

    % Add speed to legend
    lgd{i} = ['v = ', num2str(v)];
    
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
fbo.Children(3).Title.String = 'From: Steering Rate  To: Roll Rate';
fny.Children(2).Title.String = 'From: Steering Rate  To: Roll Rate';
fni.Children(2).Title.String = 'From: Steering Rate  To: Roll Rate';
figure(fpz);legend(lgd)
figure(fbo);legend(lgd)
figure(fny);legend(lgd)
figure(fni);legend(lgd)