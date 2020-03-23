close all

%% Step response
opt = stepDataOptions('StepAmplitude',2);
h = figure;step(linsys1,opt);
h.Children(2).Title.String = 'From: Steering Rate';h.Children(2).Title.Color=[0 0 0];
h.Children(3).Title.String = 'From: Measured Steering Angle';h.Children(3).Title.Color=[0 0 0];
h.Children(3).YLabel.String = 'To: True Steering Angle [deg]';h.Children(3).YLabel.Color=[0 0 0];
ylh = ylabel('Amplitude');
ylh.Position(1) = ylh.Position(1)-35;
h.Children(2).Position(1) = h.Children(2).Position(1)*1.02;


%% Bode plot
h = figure;
options = bodeoptions;
options.FreqUnits = 'Hz'; % or 'rad/second', 'rpm', etc.
bode(linsys1,options);
h.Children(3).Title.String = 'From: Steering Rate';h.Children(3).Title.Color=[0 0 0];
h.Children(5).Title.String = 'From: Measured Steering Angle';h.Children(5).Title.Color=[0 0 0];
h.Children(5).YLabel.String = 'To: True Steering Angle (Mag)';h.Children(5).YLabel.Color=[0 0 0];
h.Children(4).YLabel.String = 'To: True Steering Angle (Phase)';h.Children(4).YLabel.Color=[0 0 0];
ylh = ylabel('Phase');
ylh.Position(1) = ylh.Position(1)-26;
xlh = xlabel('Frequency');
xlh.Position(2) = xlh.Position(2)-15;
h.Children(3).Position(1) = h.Children(3).Position(1)*1.02;
h.Children(2).Position(1) = h.Children(2).Position(1)*1.02;