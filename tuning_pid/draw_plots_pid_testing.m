% % % Plot roll
% % figure();
% % hold on;
% % title('Roll Angle');
% % ylabel({'Roll Angle','[deg]'});
% % xlabel('Time [s]');
% % legend_string = [];
% % for index=1:size(K_comparison,1)
% %     if ~isempty(time{index}) 
% %         plot(time{index},roll_comparison{index})
% %         legend_string = [legend_string ; num2str(index)];
% %     end
% % end
% % legend(legend_string);
% % 
% % % Plot steering
% % figure();
% % hold on;
% % title('Steering Angle');
% % ylabel({'Steering Angle','[deg]'});
% % xlabel('Time [s]');
% % legend_string = [];
% % for index=1:size(K_comparison,1)
% %     if ~isempty(time{index}) 
% %         plot(time{index},steering_comparison{index})
% %         legend_string = [legend_string ; num2str(index)];
% %     end
% % end
% % legend(legend_string);
% % 
% % % Plot roll rate
% % figure();
% % hold on;
% % title('Roll Rate');
% % ylabel({'Roll Rate','[deg/s]'});
% % xlabel('Time [s]');
% % legend_string = [];
% % for index=1:size(K_comparison,1)
% %     if ~isempty(time{index}) 
% %         plot(time{index},rollrate_comparison{index})
% %         legend_string = [legend_string ; num2str(index)];
% %     end
% % end
% % legend(legend_string);


% Plot step response
figure();
hold on;
title('Step Response');
% ylabel({'Roll Rate','[deg/s]'});
xlabel('Time [s]');
plot(time{end},stepinput_comparison{end});
legend_string = ['S'];
for index=1:size(K_comparison,1)
    if ~isempty(time{index}) 
        plot(time{index},stepoutput_comparison{index})
        legend_string = [legend_string ; num2str(index)];
    end
end
legend(legend_string);


% Plot bike path
figure();
hold on;
title('Bicycle path vs Reference path');
xlabel('x [m]');
ylabel('y [m]');
plot(initial_x,initial_y,'*r')
legend_string_cell{1} = 'Starting Position';
plot(X_SIGMA_comparison{index},Y_SIGMA_comparison{index});
legend_string_cell{end+1} = 'Reference path';
for index=1:size(K_comparison,1)
    if ~isempty(time{index}) 
%         p_1=plot(X_BIKE_comparison{index}, Y_BIKE_comparison{index}, 'r', 'LineWidth', 1); %-or to see the time points and lign
%         legend{end+1}=['Bike''s Back Wheel Path ' num2str(index)];

        plot(X_BIKE_comparison{index}+b_real*cos(heading_comparison{index}),Y_BIKE_comparison{index}+b_real*sin(heading_comparison{index}))
        legend_string_cell{end+1}=['Bike''s Front Wheel Path ' num2str(index)];
    end
end
plot(X_BIKE_comparison{end}, 0.2*ones(size(Y_BIKE_comparison{end})), 'g', 'LineWidth', 1); %-or to see the time points and lign
plot(X_BIKE_comparison{end}, -0.2*ones(size(Y_BIKE_comparison{end})), 'g', 'LineWidth', 1); %-or to see the time points and lign
legend_string_cell{end+1} = 'Lateral limits on the roller';
legend(legend_string_cell);