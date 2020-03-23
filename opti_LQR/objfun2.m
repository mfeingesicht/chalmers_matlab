function [cost] = objfun2(lqr_coeff)
    configuration_lqrTuning;
    Q = diag(lqr_coeff(1:3)) + squareform(lqr_coeff(4:end-1));
    R = lqr_coeff(end);
    
    assignin('base','Q',Q);
    assignin('base','R',R);
    
    
    %% RUN THE SIMULINK SIMULATION

    try 
        lqr_design;
        assignin('base','lqr_gains',lqr_gains);
        sim('bike_model_lqrTuning.slx');
    catch Error_Reason
        cost = 1e9;
    end
    
    try
        if all(abs(rad2deg(steering_rate.Data))<10)
            cost = sqrt(sum((states_true.data(:,1)).^2) + sum((states_true.data(:,2)).^2) + sum((states_true.data(:,3)).^2));
        else
            cost = 1e9;
        end
    catch
        cost = 1e9;
    end
end

