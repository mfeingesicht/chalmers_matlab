close all

%% Step response
opt = stepDataOptions('StepAmplitude',2); % deg
h = figure;step(linsys1,opt);
h.Children(13).Title.String = 'From: Measured Roll';h.Children(13).Title.Color=[0 0 0];
h.Children(10).Title.String = 'From: Measured Roll Rate';h.Children(10).Title.Color=[0 0 0];
h.Children(7).Title.String = 'From: Measured Steering Angle';h.Children(7).Title.Color=[0 0 0];
h.Children(4).Title.String = 'From: True Steering Angle';h.Children(4).Title.Color=[0 0 0];
h.Children(11).YLabel.String = {'To: True Steering Angle', '[deg]'};h.Children(13).YLabel.Color=[0 0 0];
h.Children(13).YLabel.String = {'To: True Roll', '[deg/s]'};h.Children(12).YLabel.Color=[0 0 0];
h.Children(12).YLabel.String = {'To: True Roll Rate', '[deg]'};h.Children(11).YLabel.Color=[0 0 0];
ylh = ylabel('Amplitude');
ylh.Position(1) = ylh.Position(1)-35;


%% Bode plot
h = figure;
options = bodeoptions;
options.FreqUnits = 'Hz'; % or 'rad/second', 'rpm', etc.
bode(linsys1,options);
h.Children(25).Title.String = 'From: Measured Roll';h.Children(25).Title.Color=[0 0 0];
h.Children(19).Title.String = 'From: Measured Roll Rate';h.Children(19).Title.Color=[0 0 0];
h.Children(13).Title.String = 'From: Measured Steering Angle';h.Children(13).Title.Color=[0 0 0];
h.Children(7).Title.String = 'From: True Steering Angle';h.Children(7).Title.Color=[0 0 0];
h.Children(21).YLabel.String = {'To: True Steering', 'Angle (Mag)'};h.Children(21).YLabel.Color=[0 0 0];
h.Children(20).YLabel.String = {'To: True Steering', 'Angle (Phase)'};h.Children(20).YLabel.Color=[0 0 0];
h.Children(25).YLabel.String = {'To: True', 'Roll (Mag)'};h.Children(25).YLabel.Color=[0 0 0];
h.Children(24).YLabel.String = {'To: True', 'Roll (Phase)'};h.Children(24).YLabel.Color=[0 0 0];
h.Children(23).YLabel.String = {'To: True Roll', 'Rate (Mag)'};h.Children(23).YLabel.Color=[0 0 0];
h.Children(22).YLabel.String = {'To: True Roll', 'Rate (Phase)'};h.Children(22).YLabel.Color=[0 0 0];
ylh = ylabel('Phase');
ylh.Position(1) = ylh.Position(1)-26;
xlh = xlabel('Frequency');
xlh.Position(2) = xlh.Position(2)-15;