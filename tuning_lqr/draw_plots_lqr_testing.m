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

% Plot steering rate
figure();
hold on;
title('Steering Rate');
ylabel({'Steering Rate','[deg/s]'});
xlabel('Time [s]');
legend_string = [];
for index=1:size(Q_comparison,1)
    if ~isempty(time{index}) 
        plot(time{index},steeringrate_comparison{index})
        legend_string = [legend_string ; num2str(index)];
    end
end
legend(legend_string);


% True States Plot
figure();
hold on;
for index=1:size(Q_comparison,1)
    subplot(3,1,1)
    plot(time{index},roll_comparison{index})
    %axis([0 80 -3 0.5])
    hold on
    grid on
    title('Roll Angle')
    leg1=legend('$\varphi$');
    set(leg1,'Interpreter','latex');
    ylabel({'Roll Angle','[deg]'})
    xlabel('Time [s]')
    subplot(3,1,2)
    plot(time{index},steering_comparison{index})
    %axis([0 80 -0.5 6])
    hold on
    grid on
    title('Steering Angle')
    leg2=legend('$\delta$');
    set(leg2,'Interpreter','latex');
    ylabel({'Steering Angle','[deg]'})
    xlabel('Time [s]')
    subplot(3,1,3)
    plot(time{index},rollrate_comparison{index})
    %axis([0 80 -3 3.5])
    hold on
    grid on
    title('Roll Rate')
    leg3=legend('$\dot\varphi$');
    set(leg3,'Interpreter','latex');
    ylabel({'Roll Rate','[deg/s]'})
    xlabel('Time [s]')
end
