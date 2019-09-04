function [cost] = objfun(pid_coeff)
    Kp_dir = pid_coeff(1);
    Ki_dir = pid_coeff(2);
    Kd_dir = pid_coeff(3);
    Kp_pos = pid_coeff(4);
    Ki_pos = pid_coeff(5);
    Kd_pos = pid_coeff(6);
    
    assignin('base','Kp_dir',Kp_dir);
    assignin('base','Ki_dir',Ki_dir);
    assignin('base','Kd_dir',Kd_dir);
    assignin('base','Kp_pos',Kp_pos);
    assignin('base','Ki_pos',Ki_pos);
    assignin('base','Kd_pos',Kd_pos);
    
    
    %% RUN THE SIMULINK SIMULATION

    try
        sim('bike_model_pidTuning.slx');
        cost = norm(Y_SIGMA.data - Y_BIKE.data);
    catch Error_Reason
        cost = 1e9;
    end
    
%     try
% % % %         cost = sqrt(sum((states_true.data(:,1)).^2) + sum((states_true.data(:,2)).^2) + sum((states_true.data(:,3)).^2));
%         cost = norm(Y_SIGMA.data - Y_BIKE.data);
%     catch
%         cost = 1e9;
%     end
end

