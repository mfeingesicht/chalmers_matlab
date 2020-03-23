% Bike Position vs Path
if Bike_Path_vs_Reference_Path==1
    figure()
    title('Bicycle path vs Reference path')
    xlabel('x [m]')
    ylabel('y [m]')
    hold on

    p_1=plot(X_BIKE.data, Y_BIKE.data, 'r', 'LineWidth', 1); %-or to see the time points and lign
    plot(X_BIKE.Data+b_real*cos(heading.Data),Y_BIKE.Data+b_real*sin(heading.Data))
    plot(initial_x,initial_y,'*r')
    plot(X_BIKE.data, 0.2*ones(size(Y_BIKE.data)), 'g', 'LineWidth', 1); %-or to see the time points and lign
    plot(X_BIKE.data, -0.2*ones(size(Y_BIKE.data)), 'g', 'LineWidth', 1); %-or to see the time points and lign
    legend('Bike''s Back Wheel Path', 'Bike''s Front Wheel Path', 'Starting Position','Lateral limits on the roller')

    hold off
    grid on
end


% Comparison Kalman / real position
if Kalman_vs_Real==1
    figure()
    title('Error of the Kalman position estimation compared to the Bicycle path')
    xlabel('Time [s]')
    ylabel('Error [m]')
    hold on
    error_x = X_BIKE.data - X_BIKE_estim.data;
    error_y = Y_BIKE.data - Y_BIKE_estim.data;
    plot(X_BIKE_estim.time, error_x, 'r')
    plot(X_BIKE_estim.time, error_y, 'b')
    grid on
    legend('Error on the position in x direction', 'Error on the position in y direction')
    hold off
end


% % % % Errors between estimations and real position
% % % if Estimations_errors==1
% % %     figure()
% % %     hold on
% % %     title('Euclidean distance of the position estimations with regards to the Bicycle path')
% % %     xlabel('Time [s]')
% % %     ylabel('Error [m]')
% % %     error_direct_pos = ((X_BIKE.data-pos_meas.data(:,1)).^2+(Y_BIKE.data-pos_meas.data(:,2)).^2).^0.5;
% % %     error_vel_int = ((X_BIKE.data-pos_vel_int.data(:,1)).^2+(Y_BIKE.data-pos_vel_int.data(:,2)).^2).^0.5;
% % %     error_kalman = ((X_BIKE.data - X_BIKE_estim.data).^2+(Y_BIKE.data - Y_BIKE_estim.data).^2).^0.5;
% % %     plot(X_BIKE_estim.time, error_direct_pos, 'g')
% % %     plot(X_BIKE_estim.time, error_vel_int, 'b')
% % %     plot(X_BIKE_estim.time, error_kalman, 'k')
% % %     grid on
% % %     legend('Error of direct position measurement', 'Error of velocity integration estimation', 'Error of Kalman filter estimation')
% % %     hold off
% % % end


% True States Plot
if States_True==1
    figure()
    subplot(3,1,1)
    plot(states_true.time,states_true.data(:,1)*57.2958, 'b')
    %axis([0 80 -3 0.5])
    hold on
    grid on
    title('Roll Angle')
    leg1=legend('$\varphi$');
    set(leg1,'Interpreter','latex');
    ylabel({'Roll Angle','[deg]'})
    xlabel('Time [s]')
    subplot(3,1,2)
    plot(states_true.time,states_true.data(:,2)*57.2958, 'b')
    %axis([0 80 -0.5 6])
    hold on
    grid on
    title('Steering Angle')
    leg2=legend('$\delta$');
    set(leg2,'Interpreter','latex');
    ylabel({'Steering Angle','[deg]'})
    xlabel('Time [s]')
    subplot(3,1,3)
    plot(states_true.time,states_true.data(:,3)*57.2958, 'b')
    %axis([0 80 -3 3.5])
    hold on
    grid on
    title('Roll Rate')
    leg3=legend('$\dot\varphi$');
    set(leg3,'Interpreter','latex');
    ylabel({'Roll Rate','[deg/s]'})
    xlabel('Time [s]')
end



% Bike Path Error Plot
if Bike_Path_Error_Inscpection==1
    figure()
    subplot(2,1,1)
    plot(X_SIGMA, '.b')
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
    plot(Y_SIGMA, '.b')
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
end