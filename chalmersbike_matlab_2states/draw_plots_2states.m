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
%             plot(pos_vel_int.data(:,1), pos_vel_int.data(:,2), 'c')
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


% True States Plot
if States_True==1
    figure()
    subplot(3,1,1)
    plot(states_true.time,states_true.data(:,1))
    %axis([0 80 -3 0.5])
    hold on
    grid on
%     plot(states_measured.time,states_measured.data(:,1))
    try
        plot(states_measured.time,states_measured.Data(:,1))
    catch
        plot(states_measured.time,squeeze(states_measured.Data(1,:,:)))
    end
    title('State 1')
    leg1=legend('$x_1$');
    set(leg1,'Interpreter','latex');
    ylabel('State 1')
    xlabel('Time [s]')
    
    subplot(3,1,2)
    plot(states_true.time,states_true.data(:,2))
    hold on
    grid on
    try
        plot(states_measured.time,states_measured.Data(:,2))
    catch
        plot(states_measured.time,squeeze(states_measured.Data(2,:,:)))
    end
    title('State 2')
    leg2=legend('$x_2$');
    set(leg2,'Interpreter','latex');
    ylabel('State 2')
    xlabel('Time [s]')
    
    subplot(3,1,3)
    plot(calculated_input*57.2958, 'b')
    hold on
    grid on
    plot(realised_input*57.2958, 'r')
    title('Unfiltered Input vs Filtered Input')
    leg4=legend('$\delta^{unfiltered}$','$\delta^{filtered}$');
    set(leg4,'Interpreter','latex');
    ylabel({'Input: Steering',' Angle [deg]'})
    xlabel('Time [s]')
end


% Angles and Angular Rates
if Roll_RollRate_Plot==1
    figure()
    subplot(4,1,1)
    plot(roll_rollrate_true.time',roll_rollrate_true.data(:,1)*57.2958, 'b')
    hold on
    grid on
    plot(measured_roll_rollrate.time,measured_roll_rollrate.data(:,1)*57.2958, 'r')
    title('True Roll Angle vs Measured Roll Angle')
    leg1=legend('$\varphi^{true}$','$\varphi^{measured}$');
    set(leg1,'Interpreter','latex');
    ylabel({'Roll Angle','[deg]'})
    xlabel('Time [s]')
    
    subplot(4,1,2)
    plot(roll_rollrate_true.time',roll_rollrate_true.data(:,2)*57.2958, 'b')
    hold on
    grid on
    plot(measured_roll_rollrate.time,measured_roll_rollrate.data(:,2)*57.2958, 'r')
    title('True Roll Rate vs Measured Roll Rate')
    leg3=legend('$\dot\varphi^{true}$','$\dot\varphi^{measured}$');
    set(leg3,'Interpreter','latex');
    ylabel({'Roll Rate','[deg/s]'})
    xlabel('Time [s]')
    
    subplot(4,1,3)
    plot(calculated_input.time',calculated_input.data*57.2958)
    hold on
    grid on
    plot(realised_input.time',realised_input.data*57.2958)
    title('Calculated Steering Angle vs Measured Steering Angle')
    leg3=legend('$\delta^{calc}$','$\delta^{measured}$');
    set(leg3,'Interpreter','latex');
    ylabel({'Steering Angle','[deg]'})
    xlabel('Time [s]')
    
    subplot(4,1,4)
    plot(calculated_steeringrate.time',calculated_steeringrate.data*57.2958)
    hold on
    grid on
    plot(realised_steeringrate.time',realised_steeringrate.data*57.2958)
    title('Calculated Steering Rate vs Measured Steering Rate')
    leg3=legend('$\dot\delta^{calc}$','$\dot\delta^{measured}$');
    set(leg3,'Interpreter','latex');
    ylabel({'Steering Rate','[deg/s]'})
    xlabel('Time [s]')
end


% Bike Path Error Plot
if Bike_Path_Error_Inscpection==1
    figure()
    subplot(4,1,1)
    plot(X_SIGMA, 'b')
    hold on
    grid on
    plot(X_BIKE, 'r')
    title('Reference Path vs Bike Path in X Axis')
    legend('Reference Path in X','Bike Path in X')
    lege1=legend('Reference Path in X','Bike Path in X');
    ylabel('Distance [m]')
    xlabel('Time [s]')
    subplot(4,1,2)
    plot(Y_SIGMA, 'b')
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