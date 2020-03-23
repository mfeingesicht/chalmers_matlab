%% Parameter reading


STEERING_ANGLE_SIGN_INVERT = 0; % 1 for Umur Exp, 0 for RightHand frame
Gyro_ax_noise = 0.1; % deg/sec, the range of the noises

Seperate_roll_show = 1;
m = 1;      % A aribitray mass for energy visualization
g = 9.82;   % gravity
h = 0.586;          % height of center of mass [m]
b = 1.095;          % length between wheel centers [m]
c = 0.06;           % length between front wheel contact point and the extention of the fork axis
lambda = deg2rad(70); % angle of the fork axis
a = b-0.77;         % distance from rear wheel to center of mass [m]
IMU_height=0.43;     % IMU height [m]    0.96 vs 0.90 vs 0.45
D = m*a*h; % The inertia in yaw direction
J = m*h^2; % The inertia in roll direction
hallsensor_pulse_dist = 0.4398; %(pi*d/#magnets*tyre_ratio/d)
% Extract the data from the record structure
Data_length = length(Data_exp.Time);% The length of experiment sample data
if STEERING_ANGLE_SIGN_INVERT == 1  % In Umur's experiment, Steering was inverted
    % PHI counterclockwise positive, delta(after inversion) counter
    % clockwise positive
    ExpDelta = -Data_exp.Delta;
    Exp_ControlInput = -Data_exp.ControlInput;
    Phi_sign = 1; % Signs for Umur's setting, where phi direction was wrong
else % The setting consist with  Mathematical model
    % PHI clockwise positive delta counterclockwise positive consistent
    % with the right hand rule
    ExpDelta = Data_exp.Delta;
    Exp_ControlInput = Data_exp.ControlInput;
    Phi_sign = -1; % The signs for right hand rule model
end
MeasuredLongitudinalVel = Data_exp.MeasuredVelocity;
yaw_v = MeasuredLongitudinalVel.*sin(lambda).*sin(ExpDelta)/b; % The angular velocity in yaw axis
Phi_dot = Data_exp.phi_dot;
ExpTime = Data_exp.Time;

ExpPhi = steering_comparison{1};
Exp_ay = Data_exp.ay;
Exp_Phidot = Data_exp.PhiDot;
Exp_az = Data_exp.az;
Exp_ax = Data_exp.a_x;
Exp_phi_roll_comp = Data_exp.phi_roll_comp;

Fs = 25; % Hz
Ts = 1/Fs; % \appro 0.04 s/sample
% #Samples = Data_length Timestamps = ExpTime

Gyro_Error_abs = deg2rad(0.1); % The error from gyroscope reading, input: deg/s
% delta_offset = -2.0041e-04;%deg2rad();
% delta_offset = deg2rad(-0.015);
% delta_offset = deg2rad(0.008);


%% Offline calculation position integration
Delta_bias = zeros(length(Data_exp.Time),1);
Delta_intbias = Delta_bias;
y_int = zeros(length(Data_exp.Time),1);
Dn_int = zeros(length(Data_exp.Time),1);
for i = 1:length(Data_exp.Time)
    
    if i == 1
        dT = Data_exp.Time(i);
        y_int(i) = Data_exp.y(i) * dT;
        if Data_exp.MeasuredVelocity(i) ~= 0
            Dn_int(i) = b / ( Data_exp.MeasuredVelocity(i) * (a +  Data_exp.MeasuredVelocity(i) * Data_exp.Time(i)/2) * Data_exp.Time(i) )* dT;
        else
            Dn_int(i) = 0;
        end
        Delta_intbias(i) = y_int(i)*Dn_int(i);
    else
        dT = (Data_exp.Time(i) - Data_exp.Time(i-1)) ;
        y_int(i) = y_int(i-1) + Data_exp.y(i) * dT;
        if Data_exp.MeasuredVelocity(i) ~= 0
            Dn_int(i) = Dn_int(i-1) + b / ( Data_exp.MeasuredVelocity(i) * (a +  Data_exp.MeasuredVelocity(i) * Data_exp.Time(i)/2) * Data_exp.Time(i) )* dT;
        else
            Dn_int(i) = 0;
        end
        
        Delta_intbias(i) = y_int(i)*Dn_int(i);
    end
    
        Delta_bias(i) = Data_exp.y(i) * b / ( Data_exp.MeasuredVelocity(i) * (a +  Data_exp.MeasuredVelocity(i) * Data_exp.Time(i)/2) * Data_exp.Time(i) );
end
delta_offset = -max(Delta_bias);

delta_offset = -0.1908;
% delta_offset = -0.3220;
delta_offset = 0.8;
delta_offset = deg2rad(delta_offset);
delta_offset = 0;

psi = zeros(length(Data_exp.Time),1);
x = zeros(length(Data_exp.Time),1);
y = zeros(length(Data_exp.Time),1);
delta_bias = zeros(length(Data_exp.Time),1);
delta_int = zeros(length(Data_exp.Time),1);
v_delta_intint = zeros(length(Data_exp.Time),1);

i = 1;
% dT = Ts;
dT = Data_exp.Time(i);
delta = Data_exp.Delta(i) + delta_offset;
    beta = atan(a/b * tan(delta));
    dpsi = Data_exp.MeasuredVelocity(i) / a * sin(beta);
    psi(i) =  dT * dpsi;
    nu = psi(i) + beta;
    dx = Data_exp.MeasuredVelocity(i) * cos(nu);
    dy = Data_exp.MeasuredVelocity(i) * sin(nu);
    v = Data_exp.MeasuredVelocity(i);
    x(i) = dx * dT;
    y(i) = dy * dT;
    delta_int(i) = Data_exp.Delta(i) * dT;
    v_delta_intint(i) = v * delta_int(i) * dT;
    
    
    
for i = 2:length(Data_exp.Time)
    dT = (Data_exp.Time(i) - Data_exp.Time(i-1)) ;
% dT = Ts;
delta = Data_exp.Delta(i) + delta_offset;
    beta = atan(a/b * tan(delta));
    dpsi = Data_exp.MeasuredVelocity(i) /a * sin(beta);
    psi(i) = psi(i-1) + dT * dpsi;
    nu = psi(i) + beta;
    dx = Data_exp.MeasuredVelocity(i) * cos(nu);
    dy = Data_exp.MeasuredVelocity(i) * sin(nu);
    x(i) = x(i-1) + dx * dT;
    y(i) = y(i-1) + dy * dT;
end



%% Plotting the online integration result.


if (sum(ismember(Data_exp.Properties.VariableNames,'x'))) % Determine if the current reading and motor speed are recorded or not
    fig_pos = figure();
    subplot(4,1,1)
    plot(Data_exp.x,Data_exp.y,x,y)
    title('bike position')
    xlabel('x /m')
    ylabel('y /m')
    legend('Bike position estimation by integration','offline calculation')
    grid on
    subplot(4,1,2)
    plot(Data_exp.Time,Data_exp.Delta .* rad2deg(1))
%     ,Data_exp.Time,Data_exp.delta_ref * rad2deg(1))
    title('Steering Angle')
    xlabel('Time /s')
    ylabel('Angle / deg')
    legend('Steering angle by encoder')
%     ,'reference Steering angle')
    grid on
    subplot(4,1,3)
    plot(Data_exp.Time,Data_exp.y,Data_exp.Time,y)
    title('Lateral position estimation')
    xlabel('Time / s')
    ylabel('distance y / m')
    legend('online Lateral position Y estimation','offline estimation')
    grid on
    if(sum(ismember(Data_exp.Properties.VariableNames,'laserranger'))) % Determine if the current reading and motor speed are recorded or not
        subplot(4,1,4)
        hold on;
        plot(Data_exp.Time,Data_exp.laserranger)
        plot(Data_exp.Time,y)
        title('Lateral position comparison')
        xlabel('Time / s')
        ylabel('distance y / m')
        legend('online Lateral position Y estimation','offline estimation')
        grid on
    end
end

