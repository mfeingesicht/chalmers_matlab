function [cost] = objfun(lqr_coeff)
    configuration_lqrTuning;
    Q = diag(lqr_coeff(1:3));
    R = lqr_coeff(4);
    
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
        cost = sqrt(sum((states_true.data(:,1)).^2) + sum((states_true.data(:,2)).^2) + sum((states_true.data(:,3)).^2));
    catch
        cost = 1e9;
    end
end

