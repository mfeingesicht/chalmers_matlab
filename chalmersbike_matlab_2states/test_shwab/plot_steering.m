close all

%% Steering angle
figure;h1 = stepplot(linsys1(1,1),linsys2(1,1),linsys3(1,1),8);
opt=getoptions(h1);opt.InputLabels.FontSize=0.1;opt.OutputLabels.FontSize=0.1;setoptions(h1,opt);
ylabel('Steering Angle [deg]');
title('Step Response: Steering Angle');
legend('D = 0','D = 0.1','D = 1');


%% Steering rate
figure;h2 = stepplot(linsys1(2,1),linsys2(2,1),linsys3(2,1),8);
opt=getoptions(h2);opt.InputLabels.FontSize=0.1;opt.OutputLabels.FontSize=0.1;setoptions(h2,opt);
ylabel('Steering Rate [deg/s]');
title('Step Response: Steering Rate');
legend('D = 0','D = 0.1','D = 1');


%% Bode plot
figure;h3 = bodeplot(linsys1(1,[1:2]),linsys2(1,[1:2]),linsys3(1,[1:2]));
hh = gcf;
% % opt=getoptions(h3);opt.InputLabels.FontSize=0.1;opt.OutputLabels.FontSize=0.1;setoptions(h3,opt);
% % ylabel('Steering Rate [deg/s]');
% % title('Step Response: Steering Rate');
hh.Children(5).YLabel.String = 'To: Steering Angle (Mag)';
hh.Children(4).YLabel.String = 'To: Steering Angle (Phase)';
hh.Children(5).Title.String='From: Steering Angle Reference';
hh.Children(3).Title.String='From: Steering Rate Perturnation';
legend('D = 0','D = 0.1','D = 1');