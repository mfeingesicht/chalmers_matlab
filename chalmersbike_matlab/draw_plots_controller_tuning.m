%% True States Plot
figure()
subplot(3,1,1)
plot(states_true.time,states_true.data(:,1)*57.2958, 'b')
%axis([0 80 -3 0.5])
hold on
grid on
title('True Roll Angle')
leg1=legend('$\varphi$');
set(leg1,'Interpreter','latex');
ylabel({'Roll Angle','[deg]'})
xlabel('Time [s]')
subplot(3,1,2)
plot(states_true.time,states_true.data(:,2)*57.2958, 'b')
%axis([0 80 -0.5 6])
hold on
grid on
title('True Steering Angle')
leg2=legend('$\delta$');
set(leg2,'Interpreter','latex');
ylabel({'Steering Angle','[deg]'})
xlabel('Time [s]')
subplot(3,1,3)
plot(states_true.time,states_true.data(:,3)*57.2958, 'b')
%axis([0 80 -3 3.5])
hold on
grid on
title('True Roll Rate')
leg3=legend('$\dot\varphi$');
set(leg3,'Interpreter','latex');
ylabel({'Roll Rate','[deg/s]'})
xlabel('Time [s]')

set(findall(gcf, 'Type', 'Line'),'LineWidth',2);set(findobj(gcf,'type','axes'),'FontSize',15);


%% Measured States Plot
figure()
subplot(3,1,1)
plot(measured_states.time,measured_states.data(:,1)*57.2958, 'b')
%axis([0 80 -3 0.5])
hold on
grid on
title('Measured Roll Angle')
leg1=legend('$\varphi$');
set(leg1,'Interpreter','latex');
ylabel({'Roll Angle','[deg]'})
xlabel('Time [s]')
subplot(3,1,2)
plot(measured_states.time,measured_states.data(:,2)*57.2958, 'b')
%axis([0 80 -0.5 6])
hold on
grid on
title('Measured Steering Angle')
leg2=legend('$\delta$');
set(leg2,'Interpreter','latex');
ylabel({'Steering Angle','[deg]'})
xlabel('Time [s]')
subplot(3,1,3)
plot(measured_states.time,measured_states.data(:,3)*57.2958, 'b')
%axis([0 80 -3 3.5])
hold on
grid on
title('Measured Roll Rate')
leg3=legend('$\dot\varphi$');
set(leg3,'Interpreter','latex');
ylabel({'Roll Rate','[deg/s]'})
xlabel('Time [s]')

set(findall(gcf, 'Type', 'Line'),'LineWidth',2);set(findobj(gcf,'type','axes'),'FontSize',15);


%% Plot bike path
clear legend_string_cell
figure();
hold on;
title('Bicycle path vs Reference path');
xlabel('x [m]');
ylabel('y [m]');
plot(initial_x,initial_y,'*r')
legend_string_cell{1} = 'Starting Position';
plot(X_SIGMA.Data,Y_SIGMA.Data);
legend_string_cell{end+1} = 'Reference path';
plot(pos_meas.Data(:,1),pos_meas.Data(:,2));
legend_string_cell{end+1} = 'Measured position';
plot(X_BIKE.Data+b_real*cos(heading.Data),Y_BIKE.Data+b_real*sin(heading.Data))
legend_string_cell{end+1}='Bike''s Front Wheel Path';
plot(X_BIKE.Data, 0.2*ones(size(Y_BIKE.Data)), 'g', 'LineWidth', 1); %-or to see the time points and lign
plot(X_BIKE.Data, -0.2*ones(size(Y_BIKE.Data)), 'g', 'LineWidth', 1); %-or to see the time points and lign
legend_string_cell{end+1} = 'Lateral limits on the roller';
legend(legend_string_cell);

set(findall(gcf, 'Type', 'Line'),'LineWidth',2);set(findobj(gcf,'type','axes'),'FontSize',15);




%% Bike Path Error Plot
figure()
subplot(2,1,1)
plot(X_SIGMA, 'b')
hold on
grid on
plot(X_BIKE, 'r')
plot(X_BIKE.Time,X_BIKE.Data+b_real*cos(heading.Data))
title('Reference Path vs Bike Path in X Axis')
legend('Reference Path in X','Bike Path in X')
lege1=legend('Reference Path in X','Bike''s Back Wheel Path in X','Bike''s Front Wheel Path in X');
ylabel('Distance [m]')
xlabel('Time [s]')
subplot(2,1,2)
plot(Y_SIGMA, 'b')
hold on
grid on
plot(Y_BIKE, 'r')
plot(Y_BIKE.Time,Y_BIKE.Data+b_real*sin(heading.Data))
plot(Y_BIKE.Time,0.2*ones(size(Y_BIKE.Data)),'g');
plot(Y_BIKE.Time,-0.2*ones(size(Y_BIKE.Data)),'g');
title('Reference Path vs Bike Path in Y Axis')
lege2=legend('Reference Path in Y','Bike''s Back Wheel Path in Y','Bike''s Front Wheel Path in Y','Lateral limits on the roller');
ylabel('Distance [m]')
xlabel('Time [s]')