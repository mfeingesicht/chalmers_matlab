%% Bode plot
opt=bodeoptions;opt.FreqUnits='Hz';
figure;h = bodeplot(linsys3,opt);
hh = gcf;
hh.Children(3).YLabel.String = 'To: Steering Angle (Mag)';
hh.Children(2).YLabel.String = 'To: Steering Angle (Phase)';
hh.Children(3).Title.String='From: Steering Angle Reference';
