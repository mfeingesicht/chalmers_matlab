% Plot roll
figure();
hold on;
title('Roll Angle');
ylabel({'Roll Angle','[deg]'});
xlabel('Time [s]');
legend_string = [];
for index=1:size(Q_comparison,1)
    if ~isempty(time{index}) 
        plot(time{index},roll_comparison{index})
        legend_string = [legend_string ; num2str(index)];
    end
end
legend(legend_string);

% Plot steering
figure();
hold on;
title('Steering Angle');
ylabel({'Steering Angle','[deg]'});
xlabel('Time [s]');
legend_string = [];
for index=1:size(Q_comparison,1)
    if ~isempty(time{index}) 
        plot(time{index},steering_comparison{index})
        legend_string = [legend_string ; num2str(index)];
    end
end
legend(legend_string);

% Plot roll rate
figure();
hold on;
title('Roll Rate');
ylabel({'Roll Rate','[deg/s]'});
xlabel('Time [s]');
legend_string = [];
for index=1:size(Q_comparison,1)
    if ~isempty(time{index}) 
        plot(time{index},rollrate_comparison{index})
        legend_string = [legend_string ; num2str(index)];
    end
end
legend(legend_string);