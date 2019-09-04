%% Path tracking performance indicators
if path_tracking_indicator==1
    % Velocity tracking
        vel_error=0;
        for i = 1:vel_meas.Time(end)
            vel_error = vel_error + abs(input_velocity - vel_meas.Data(i));
        end
        vel_indicator = vel_error/vel_meas.Time(end)
    %Path tracking - average absolute error
        %x_error(X_BIKE.Time(end),1)=0;
        %y_error(X_BIKE.Time(end),1)=0;
        x_error_tot=0;
        y_error_tot=0;
        N = length(X_BIKE.Data);
        for i = 1:N
            x_error(i,1) = abs(X_SIGMA.Data(i) - X_BIKE.Data(i));
            x_error_tot = x_error_tot + x_error(i,1);
            y_error(i,1) = abs(Y_SIGMA.Data(i) - Y_BIKE.Data(i));
            y_error_tot = y_error_tot + y_error(i,1);
        end
        x_abs_error_indicator = x_error_tot/N
        y_abs_error_indicator = y_error_tot/N
    %Maximum error
        x_error_max_indicator = max(x_error)
        y_error_max_indicator = max(y_error)
%     %Standard deviation
%         %x_error_dev(X_BIKE.Time(end),1)=0;
%         %y_error_dev(X_BIKE.Time(end),1)=0;
%         x_error_tot_dev=0;
%         y_error_tot_dev=0;
%         for i = 1:N
%             x_error_dev(i,1) = x_error(i,1)^2;
%             x_error_tot_dev = x_error_tot_dev + x_error_dev(i,1);
%             y_error_dev(i,1) = y_error(i,1)^2;
%             y_error_tot_dev = y_error_tot_dev + y_error_dev(i,1);
% 
%         end
%         x_st_dev_indicator = (x_error_tot_dev/N)^(1/2)
%         y_st_dev_indicator = (y_error_tot_dev/N)^(1/2)

        %Euclidean distance
        eucl_dist_tot=0;
        N = length(X_BIKE.Data);
        for i = 1:N
            eucl_dist(i,1) = ((X_SIGMA.data(i) - X_BIKE_estim.data(i)).^2+(Y_SIGMA.data(i) - Y_BIKE_estim.data(i)).^2).^0.5;
            eucl_dist_tot = eucl_dist_tot + eucl_dist(i,1);
        end
        eucl_dist_ind = eucl_dist_tot/N
end