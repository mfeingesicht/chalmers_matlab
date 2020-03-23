% Draw bike animation
if Bike_animation==1
    figure()
    
    hold on
    
    if path_tracking==1
        sgtitle(['Path Tracking Test - Path Nr.', num2str(path)])
        subplot(1,2,1);
        plot(initial_x,initial_y,'*r')      %Gives the initial position of the bike
        h = animatedline('Color', [1 0 0]); %Line built point by point of the real path %red
        hr = animatedline('Color', [0 0 1]);%Line built point by point of the reference path %blue
        if path == 1
            axis([0 250 -3 3])            %Axis definition for the path simulation
        elseif path == 2
            axis([-40 40 -30 30])           %Different axis for the chosen path to see the results better
            axis equal
        elseif path ==3
            axis([-10 300 -2 30])
        elseif path == 4
            axis([-40 40 -30 30])
            axis equal
        elseif path ==6
            axis([-10 300 -20 60])
        elseif path ==10
            axis([-50 150 -100 40])
        elseif path ==11
            axis([-50 150 -100 40])
        elseif path ==12
            axis([-50 150 -100 40])
        else
            axis([-50 150 -40 40])
        end
        x = X_BIKE.data;                %Real path
        xr = X_SIGMA.data;              %Reference path
        
        %(jumps of scaleP time steps at a time)
        %The higher the number of scaleP, the faster is the simulation
        % =8 for real time simulation
        
        for k = 1:scaleP:length(x)  %k = 1, 1+scaleP, 1+2*scaleP, ... length(x)
            
            %Path simulation
            subplot(1,2,1);         %To see both simulations (path and 3D) on same figure
            grid on
            xlabel('x [m]')
            ylabel('y [m]')
            title ('Path simulation')
            legend('Bike Initial Position','Bike Path', 'Reference Path')
            
            if k == 1                             %We consider every time step (see 'Ts' value)
                y = Y_BIKE.data(k);
                yr = Y_SIGMA.data(k);
                addpoints(h,x(k),y);              %Draws the graph point by point - real path
                addpoints(hr,xr(k),yr);           %                               - reference path
            else                                  %We consider 'scaleP' points at a time (see 'scaleP' value)
                y = Y_BIKE.data(k-scaleP+1:k);    %Gives the y coordinate of 'scaleP' points at a time
                yr = Y_SIGMA.data(k);
                addpoints(h,x(k-scaleP+1:k),y);   %Draws the graph 'scaleP' points at a time
                addpoints(hr,xr(k),yr);           %Directly compared to the reference path
            end
            
            drawnow         %Draws the graph at this moment precisely
            %(no waiting for the whole calculation)
            
            %3D simulation
            subplot(1,2,2);
            
            if k == 1
                b = bike3DShow();        %Model of the bike and update of the drawing
            end
            
            Da.time = measured_states.Time(k);
            %Position of the measured states translated to their defined
            %position in 3Dmodel
            % -> States in the 3Dsimulation : 1-Roll angle, 2-Roll velocity,
            % 3-Steering angle, 4-Steering velocity, 5-Velocity
            Da.signals.values = zeros(1,5);                         %Initialization states values
            Da.signals.values(:,1) = measured_states.Data(k,1);     %Measured roll angle
            Da.signals.values(:,3) = measured_states.Data(k,2);     %Measured steering angle
            Da.signals.values(:,5) = measured_velocity.Data(k);     %Measured velocity
            b.fromData(Da);
            
        end
        hold off    %Keeps the last image of the path and the 3Dsimulation
        
    else
        sgtitle(['Self-Balancing Test'])
        subplot(1,2,1);
        plot(initial_x,initial_y,'*r') %Gives the initial position of the bike
        h = animatedline('Color', [0 0 1]);           %Line that will be built point by point of the path
        axis equal                  %Axis definition for the path simulation
        
        x = X_BIKE.data;            %Used to determine the length of the data
        
        %(jumps of scaleP time steps at a time)
        %The higher the number of scaleP, the
        %higher the speed of the simulation
        % =8 for real time simulation
        
        for k = 1:scaleP:length(x)  %k = 1, 1+scaleP, 1+2*scaleP, ... length(x)
            
            %Path simulation
            subplot(1,2,1);         %We want to see both simulations on the same figure
            grid on
            xlabel('x [m]')
            ylabel('y [m]')
            title ('Path simulation')
            
            if k == 1                               %We consider every time step (see 'Ts' value)
                y = Y_BIKE.data(k);
                addpoints(h,x(k),y);                %Draws the graph point by point
            else                                    %We consider 'scaleP' points at a time (
                %see 'scaleP' value)
                y = Y_BIKE.data(k-scaleP+1:k);      %Gives the y coordinate of 'scaleP' points at a time
                addpoints(h,x(k-scaleP+1:k),y);     %Draws the graph 'scaleP' point at a time
            end
            
            drawnow         %Draws the graph at this moment precisely
            
            %3D simulation
            subplot(1,2,2);
            
            if k == 1
                b = bike3DShow();    %Generates the bike 3Dsimulation
            end
            
            Da.time = measured_states.Time(k);                   %Gets the time from measured states
            %Position of the measured states translated to their defined
            %position in 3Dmodel -> States in the 3Dsimulation : 1-Roll angle,
            %2-Roll velocity, 3-Steering angle, 4-Steering velocity, 5-Velocity
            a.signals.values = zeros(1,5);                         %Initialization of the states values
            Da.signals.values(:,1) = measured_states.Data(k,1);  %Measured roll angle (1st of measured states)
            Da.signals.values(:,3) = measured_states.Data(k,2);  %Measured steering angle (2nd of measured states)
            Da.signals.values(:,5) = measured_states.Data(k,4);  %Measured velocity (4st of measured states)
            b.fromData(Da);
            
        end
        hold off    %Keeps the last image of the path and the 3Dsimulation
    end
end


% % % %
% % % if Bike_path_vs_ref_axis_equal==1
% % %     figure()
% % %     title('Bicycle path vs Reference path')
% % %     xlabel('x [m]')
% % %     ylabel('y [m]')
% % %     hold on
% % %     if Bike_path_estimations==1% Bike position estimations
% % %         %(measurement vs integration of velocity vs kalman filter)
% % %         plot(pos_meas.data(:,1), pos_meas.data(:,2), 'g')
% % %         plot(X_BIKE_estim.data, Y_BIKE_estim.data, 'k')
% % %         plot(pos_vel_int.data(:,1), pos_vel_int.data(:,2), 'c')
% % %         plot(X_SIGMA.data, Y_SIGMA.data, 'b','LineWidth', 2)
% % %         p_1=plot(X_BIKE.data, Y_BIKE.data, 'r', 'LineWidth', 1); %-or to see the time points and lign
% % %         
% % %         plot(initial_x,initial_y,'*r')
% % %         grid on
% % %         axis equal
% % %         legend('Position Measurement Path' , 'Kalman Path Estimation','Integration of Velocity Path','Reference Path', 'Bicycle Path','Starting Position')
% % %     else
% % %         %plot(X_BIKE_estim.data, Y_BIKE_estim.data, 'c', 'LineWidth', 1) %'Kalman Path Estimation',
% % %         plot(X_SIGMA.data, Y_SIGMA.data, 'b','LineWidth', 2)
% % %         p_1=plot(X_BIKE.data, Y_BIKE.data, 'r', 'LineWidth', 1); %-or to see the time points and lign
% % %         plot(initial_x,initial_y,'*r')
% % %         grid on
% % %         legend('Reference Path', 'Bicycle Path', 'Starting Position')
% % %         axis equal
% % %     end
% % % end


% Bike Position vs Path
if Bike_Path_vs_Reference_Path==1
    figure()
    title('Bicycle path vs Reference path')
    xlabel('x [m]')
    ylabel('y [m]')
    hold on
    
    if path_tracking==1
        
        if Bike_path_estimations==1% Bike position estimations
            %(measurement vs integration of velocity vs kalman filter)
%             plot(pos_meas.data(:,1), pos_meas.data(:,2), 'g')
            plot(X_BIKE_estim.data, Y_BIKE_estim.data, 'k')
            plot(pos_vel_int.data(:,1), pos_vel_int.data(:,2), 'c')
            plot(X_SIGMA.data, Y_SIGMA.data, 'b','LineWidth', 2)
            p_1=plot(X_BIKE.data, Y_BIKE.data, 'r', 'LineWidth', 1); %-or to see the time points and lign
            plot(initial_x,initial_y,'*r')
            grid on
%             legend('Position Measurement Path' , 'Kalman Path Estimation','Integration of Velocity Path','Reference Path', 'Bicycle Path','Starting Position')
            legend('Kalman Path Estimation','Integration of Velocity Path','Reference Path', 'Bicycle Path','Starting Position')
        else
            %plot(X_BIKE_estim.data, Y_BIKE_estim.data, 'c', 'LineWidth',
            %1) %'Kalman Path Estimation',
            plot(X_SIGMA.data, Y_SIGMA.data, 'b','LineWidth', 2)
            p_1=plot(X_BIKE.data, Y_BIKE.data, 'r', 'LineWidth', 1); %-or to see the time points and lign
            plot(initial_x,initial_y,'*r')
            legend('Reference Path', 'Bicycle Path', 'Starting Position')
        end
        
        
        if path == 1
            axis([0 250 -1 1])              %Axis definition for the path graph
        elseif path == 2
            axis([-40 40 -30 30])           %Different axis for the chosen path to see the results better
            axis equal
        elseif path ==3
            axis([-10 300 -2 30])
        elseif path == 4
            axis([-40 40 -30 30])
            axis equal
        elseif path ==6
            axis([-10 300 -20 60])
        elseif path ==10
            axis([-50 50 -100 40])
        elseif path ==11
            axis([-50 50 -100 40])
        elseif path ==12
            axis([-50 50 -40 40])
        else
            axis([-10 200 -40 40])
        end
    else
        p_1=plot(X_BIKE.data, Y_BIKE.data, 'r', 'LineWidth', 1); %-or to see the time points and lign
        plot(initial_x,initial_y,'*r')
        axis equal
        legend('Bike Path', 'Starting Position')
    end
    
    hold off
    grid on
    pbaspect([1 1 1])
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


% Errors between estimations and real position
if Estimations_errors==1
    figure()
    hold on
    title('Euclidean distance of the position estimations with regards to the Bicycle path')
    xlabel('Time [s]')
    ylabel('Error [m]')
    error_direct_pos = ((X_BIKE.data-pos_meas.data(:,1)).^2+(Y_BIKE.data-pos_meas.data(:,2)).^2).^0.5;
    error_vel_int = ((X_BIKE.data-pos_vel_int.data(:,1)).^2+(Y_BIKE.data-pos_vel_int.data(:,2)).^2).^0.5;
    error_kalman = ((X_BIKE.data - X_BIKE_estim.data).^2+(Y_BIKE.data - Y_BIKE_estim.data).^2).^0.5;
    plot(X_BIKE_estim.time, error_direct_pos, 'g')
    plot(X_BIKE_estim.time, error_vel_int, 'b')
    plot(X_BIKE_estim.time, error_kalman, 'k')
    grid on
    legend('Error of direct position measurement', 'Error of velocity integration estimation', 'Error of Kalman filter estimation')
    hold off
end


% True States Plot
if States_True==1
    figure()
    subplot(4,1,1)
    plot(states_true.time,states_true.data(:,1)*57.2958, 'b')
    %axis([0 80 -3 0.5])
    hold on
    grid on
    title('Roll Angle')
    leg1=legend('$\varphi$');
    set(leg1,'Interpreter','latex');
    ylabel({'Roll Angle','[deg]'})
    xlabel('Time [s]')
    subplot(4,1,2)
    plot(states_true.time,states_true.data(:,2)*57.2958, 'b')
    %axis([0 80 -0.5 6])
    hold on
    grid on
    title('Steering Angle')
    leg2=legend('$\delta$');
    set(leg2,'Interpreter','latex');
    ylabel({'Steering Angle','[deg]'})
    xlabel('Time [s]')
    subplot(4,1,3)
    plot(states_true.time,states_true.data(:,3)*57.2958, 'b')
    %axis([0 80 -3 3.5])
    hold on
    grid on
    title('Roll Rate')
    leg3=legend('$\dot\varphi$');
    set(leg3,'Interpreter','latex');
    ylabel({'Roll Rate','[deg/s]'})
    xlabel('Time [s]')
    subplot(4,1,4)
    plot(calculated_input*57.2958, '.b')
    hold on
    grid on
    plot(realised_input*57.2958, 'r')
    %axis([0 80 -10 10])
    title('Reference Input vs Realised Input')
    leg4=legend('$\dot\delta^{ref}$','$\dot\delta^{input}$');
    set(leg4,'Interpreter','latex');
    ylabel({'Input: Steering',' Rate [deg/s]'})
    xlabel('Time [s]')
end


% Measured States Plot
if States_Plot==1
    figure()
    subplot(4,1,1)
    plot(states_true.time,states_true.data(:,1)*57.2958, '.b')
    hold on
    grid on
    plot(states_measured.time,states_measured.data(:,1)*57.2958, 'r')
    %axis([0 80 -3 0.5])
    title('True Roll Angle vs Estimated Roll Angle')
    leg1=legend('$\varphi^{true}$','$\varphi^{estimated}$');
    set(leg1,'Interpreter','latex');
    ylabel({'Roll Angle','[deg]'})
    xlabel('Time [s]')
    subplot(4,1,2)
    plot(states_true.time,states_true.data(:,2)*57.2958, '.b')
    hold on
    grid on
    plot(states_measured.time,states_measured.data(:,2)*57.2958, 'r')
    %axis([0 80 -0.5 6])
    title('True Steering Angle vs Measured Steering Angle')
    leg2=legend('$\delta^{true}$','$\delta^{measured}$');
    set(leg2,'Interpreter','latex');
    ylabel({'Steering Angle','[deg]'})
    xlabel('Time [s]')
    subplot(4,1,3)
    plot(states_true.time,states_true.data(:,3)*57.2958, '.b')
    hold on
    grid on
    plot(states_measured.time,states_measured.data(:,3)*57.2958, 'r')
    %axis([0 80 -3 3.5])
    title('True Roll Rate vs Measured Roll Rate')
    leg3=legend('$\dot\varphi^{true}$','$\dot\varphi^{measured}$');
    set(leg3,'Interpreter','latex');
    ylabel({'Roll Rate','[deg/s]'})
    xlabel('Time [s]')
    subplot(4,1,4)
    plot(calculated_input*57.2958, '.b')
    hold on
    grid on
    plot(realised_input*57.2958, 'r')
    %axis([0 80 -10 10])
    title('Reference Input vs Realised Input')
    leg4=legend('$\dot\delta^{ref}$','$\dot\delta^{input}$');
    set(leg4,'Interpreter','latex');
    ylabel({'Input: Steering',' Rate [deg/s]'})
    xlabel('Time [s]')
end


% Bike Path Error Plot
if Bike_Path_Error_Inscpection==1
    figure()
    subplot(4,1,1)
    plot(X_SIGMA, '.b')
    hold on
    grid on
    plot(X_BIKE, 'r')
    title('Reference Path vs Bike Path in X Axis')
    legend('Reference Path in X','Bike Path in X')
    lege1=legend('Reference Path in X','Bike Path in X');
    ylabel('Distance [m]')
    xlabel('Time [s]')
    subplot(4,1,2)
    plot(Y_SIGMA, '.b')
    hold on
    grid on
    plot(Y_BIKE, 'r')
    title('Reference Path vs Bike Path in Y Axis')
    lege2=legend('Reference Path in Y','Bike Path in Y');
    ylabel('Distance [m]')
    xlabel('Time [s]')
    subplot(4,1,3)
    plot(LAT_ERR)
    title('Lateral Error')
    grid on
    ylabel('Lateral Error [m]')
    xlabel('Time [s]')
    subplot(4,1,4)
    plot((PSI_S-(PSI+BETA))*57.2958)
    grid on
    title('Angular Error')
    ylabel('Angular Error [deg]')
    xlabel('Time [s]')
end