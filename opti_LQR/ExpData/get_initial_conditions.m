clear all
close all
clc


%% Create vectors to store initial conditions gathered from data
delta_initCond = [];
phi_initCond = [];
phidot_initCond = [];


%% Get list of data files
files = dir('Data/*.csv');


%% Get initial conditions from data
for i=1:length(files)
    % Read data
    Data_exp1 = readtable([files(i).folder '\' files(i).name]);
    
    try
        % Add initial conditions to initCond vectors
        delta_initCond = [delta_initCond rad2deg(Data_exp1.Delta(1))];
        phi_initCond = [phi_initCond rad2deg(Data_exp1.Phi(1))];
        phidot_initCond = [phidot_initCond rad2deg(Data_exp1.PhiDot(1))];
    end
end

fprintf(1,'\n\t\tPhi\t\t\tDelta\t\tPhiDot\n');
fprintf(1,'Mean\t%.3f\t\t%.3f\t\t%.3f\n',mean(phi_initCond),mean(delta_initCond),mean(phidot_initCond));
fprintf(1,'STD\t\t%.3f\t\t%.3f\t\t%.3f\n',std(phi_initCond),std(delta_initCond),std(phidot_initCond));
fprintf(1,'Min\t\t%.3f\t\t%.3f\t\t%.3f\n',min(phi_initCond),min(delta_initCond),min(phidot_initCond));
fprintf(1,'Max\t\t%.3f\t\t%.3f\t\t%.3f\n',max(phi_initCond),max(delta_initCond),max(phidot_initCond));



% % % 
% % % %% Process data
% % % STEERING_ANGLE_SIGN_INVERT = 0; % 1 for Umur Exp, 0 for RightHand frame
% % % Gyro_ax_noise = 0.1; % deg/sec, the range of the noises
% % % 
% % % Seperate_roll_show = 1;
% % % % if(isempty(Data_exp))
% % % %     Data_exp = readtable('BikeData-20190117-151924.csv');
% % % % end
% % % % The Bike Parameters
% % % m = 1;      % A aribitray mass for energy visualization
% % % g = 9.81;   % gravity
% % % h = 0.586;          % height of center of mass [m]
% % % b = 1.095;          % length between wheel centers [m]
% % % c = 0.06;           % length between front wheel contact point and the extention of the fork axis
% % % lambda = deg2rad(70); % angle of the fork axis
% % % a = b-0.77;         % distance from rear wheel to center of mass [m]
% % % IMU_height=0.43;     % IMU height [m]    0.96 vs 0.90 vs 0.45
% % % D = m*a*h; % The inertia in yaw direction
% % % J = m*h^2; % The inertia in roll direction
% % % hallsensor_pulse_dist = 0.4398; %(pi*d/#magnets*tyre_ratio/d)
% % % % Extract the data from the record structure
% % % Data_length = length(Data_exp.Time);% The length of experiment sample data
% % % if STEERING_ANGLE_SIGN_INVERT == 1  % In Umur's experiment, Steering was inverted
% % %     % PHI counterclockwise positive, 
% % %     % delta(after inversion) counter clockwise positive
% % %     ExpDelta = -Data_exp.Delta;
% % %     Exp_ControlInput = -Data_exp.ControlInput;
% % %     Phi_sign = 1; % Signs for Umur's setting, where phi direction was wrong
% % % else % The setting consist with  Mathematical model
% % %     % PHI clockwise positive
% % %     % delta counterclockwise positive
% % %     % consistent with the right hand rule
% % %     ExpDelta = Data_exp.Delta;
% % %     Exp_ControlInput = Data_exp.ControlInput;
% % %     Phi_sign = -1; % The signs for right hand rule model
% % % end
% % % MeasuredLongitudinalVel = Data_exp.MeasuredVelocity;
% % % yaw_v = MeasuredLongitudinalVel.*sin(lambda).*sin(ExpDelta)/b; % The angular velocity in yaw axis
% % % Phi_dot = Data_exp.phi_dot;
% % % ExpTime = Data_exp.Time;
% % % 
% % % ExpPhi = Data_exp.Phi;
% % % Exp_ay_roll_comp = Data_exp.ay_roll_comp;
% % % Exp_ay = Data_exp.ay;
% % % Exp_Phidot = Data_exp.PhiDot;
% % % Exp_az = Data_exp.az;
% % % Exp_ax = Data_exp.a_x;
% % % Exp_phi_roll_comp = Data_exp.phi_roll_comp;
% % % 
% % % Fs = 25; % Hz
% % % Ts = 1/Fs; % \appro 0.04 s/sample
% % % % #Samples = Data_length
% % % % Timestamps = ExpTime
% % % 
% % % Gyro_Error_abs = deg2rad(0.1); % The error from gyroscope reading, input: deg/s
% % % 
% % % 
% % % diff_LongitudinalAcc = zeros(Data_length,1);
% % % for i = 2:Data_length
% % %     diff_LongitudinalAcc(i) = MeasuredLongitudinalVel(i) - MeasuredLongitudinalVel(i-1);
% % % end
% % % 
% % % 
% % % %% Check the consistentcy of \delta dot
% % % Delta_estimate = zeros(Data_length,1); % Calculated from the \phi dot
% % % Delta_estimate_delay = zeros(Data_length,1); % Calculated from the \phi dot
% % % Delta_estimate(1) = ExpDelta(1);
% % % Delta_estimate_delay(1) = ExpDelta(1);
% % % for i = 2:Data_length
% % % %    Delta_estimate(i) = Delta_estimate(i-1) + Exp_ControlInput(i)*(ExpTime(i) - ExpTime(i-1));
% % %     Delta_estimate(i) = ExpDelta(i-1) + Exp_ControlInput(i-1)*(ExpTime(i) - ExpTime(i-1));
% % %     Delta_estimate_delay(i) = ExpDelta(i-1);
% % % end
% % % 
% % % Ddelta = zeros(Data_length,1); % Calculated from the \delta dot
% % % for i = 2:Data_length
% % %     Ddelta(i) = (ExpDelta(i) - ExpDelta(i-1))/(ExpTime(i) - ExpTime(i-1));
% % % end
% % % 
% % % 
% % % %% Complementary a_y check, compare it with ay_roll_comp
% % % a_y_check = zeros(Data_length,1);
% % % a_y_check(1) = Exp_ay(1);
% % % ddot_phi_old = 0;
% % % for i = 2:Data_length
% % %     ddot_phi = Exp_Phidot(i) - Exp_Phidot(i-1);
% % %     a_y_check(i) = Exp_ay(i) + IMU_height*(ddot_phi*0.2+ddot_phi_old*0.8)/(ExpTime(i) - ExpTime(i-1))/g ;
% % %     ddot_phi_old = ddot_phi;
% % % end
% % % d_real = 0.43;
% % % centripedal = (MeasuredLongitudinalVel.^2 / b) .* tan(ExpDelta * sin(lambda))/g;
% % % Delta_eff = asin(sin(ExpDelta).*sin(lambda));
% % % % Delta_eff = ExpDelta.*sin(lambda);
% % % % centripedal1 = sign(Delta_eff).*((MeasuredLongitudinalVel.^2 / b^2) .* tan(Delta_eff).^2 .* sqrt(b^2 * cot(Delta_eff).^2 + d_real^2))./g;
% % % Phi_check = zeros(Data_length,1);
% % % Phi_check(1) = Exp_phi_roll_comp(1);
% % % Phi_check1 = zeros(Data_length,1);
% % % Phi_check1(1) = Exp_phi_roll_comp(1);
% % % 
% % % 
% % % Phi_acc_check_intermediate = zeros(Data_length,1);
% % % Phi_acc_check_intermediate(1) = atan2(Exp_ay_roll_comp(1) + Phi_sign * centripedal(1)*cos(0),Exp_az(1) - Phi_sign * centripedal(1)*sin(0));
% % % Phi_acc_check_intermediate1 = zeros(Data_length,1);
% % % Phi_acc_check_intermediate1(1) = atan2(Exp_ay_roll_comp(1) ,Exp_az(1));
% % % 
% % % 
% % % phi_gyro = zeros(Data_length,1);
% % % phi_gyro(1) = Exp_phi_roll_comp(1);
% % % az_new = zeros(Data_length,1);
% % % az_new(1) =  Exp_az(i) + Exp_Phidot(i)^2 * IMU_height; % vertical centripedal force by roll rotation compensated
% % % 
% % % Phi_check2 = zeros(Data_length,1);
% % % % Phi_check2(1) = 0.015*atan2(Exp_ay_roll_comp(1) ,az_new(1)) + 0.985 *(Exp_Phidot(1) * (0.04));
% % % Phi_check2(1) = Exp_phi_roll_comp(1);
% % % 
% % % Phi_acc_check_intermediate2 = zeros(Data_length,1);
% % % Phi_acc_check_intermediate2(1) = atan2(Exp_ay_roll_comp(1) + Phi_sign *  centripedal(1)*cos(0) ,az_new(1) - Phi_sign *  centripedal(1)*sin(0));
% % % 
% % % error_gyro_curve = zeros(Data_length,1);
% % % for i = 2:Data_length
% % %     Phi_acc_check_intermediate(i) = atan2(Exp_ay_roll_comp(i) +  Phi_sign * centripedal(i)*cos(Phi_check(i-1)),Exp_az(i) - Phi_sign * centripedal(i)*sin(Phi_check(i-1)));
% % %     phi_gyro(i) = phi_gyro(i-1) + Data_exp.phi_dot(i-1) * (Data_exp.Time(i) - Data_exp.Time(i-1));
% % % %     Phi_check(i) =  0.015* Phi_acc_check_intermediate + 0.985 *(Phi_check(i-1) + Exp_Phidot(i) * (ExpTime(i) - ExpTime(i-1)));
% % %     Phi_check(i) =  0.015* Phi_acc_check_intermediate(i) + 0.985 *(Phi_check(i-1) + Exp_Phidot(i) * (0.04) );
% % %     
% % % %     Phi_acc_check_intermediate1 = atan2(Exp_ay_roll_comp(i) + centripedal1(i)*cos(Phi_check1(i-1)),Exp_az(i)-centripedal1(i)*sin(Phi_check1(i-1)));
% % %     Phi_acc_check_intermediate1(i) = atan2(Exp_ay_roll_comp(i) ,Exp_az(i));
% % %     Phi_check1(i) =  0.015* Phi_acc_check_intermediate1(i) + 0.985 *(Phi_check1(i-1) + Exp_Phidot(i) * (0.04));
% % %     
% % %     az_new(i) = Exp_az(i) + Exp_Phidot(i)^2 * IMU_height;
% % %     Phi_acc_check_intermediate2(i) = atan2(Exp_ay_roll_comp(i) +  Phi_sign * centripedal(i)*cos(Phi_check2(i-1)) ,az_new(i) -  Phi_sign * centripedal(i)*sin(Phi_check2(i-1)));
% % %     Phi_check2(i) =  0.015* Phi_acc_check_intermediate2(i) + 0.985 *(Phi_check2(i-1) + Exp_Phidot(i) * (0.04));
% % %     
% % %     % The maximal possible error by gyroscope
% % %     error_gyro_curve(i) = error_gyro_curve(i-1) + Gyro_Error_abs * (ExpTime(i) - ExpTime(i-1)); % rad
% % % 
% % % end
% % % 
% % % 
% % % 
% % % % if the direct Gyro Integration by IMU appears
% % % if (sum(ismember(Data_exp.Properties.VariableNames,'phi_Gyro_intArd'))) 
% % %     phi_Gyro_intArd_rad = deg2rad(Data_exp.phi_Gyro_intArd); % in rad/s
% % %     % Correct the gyro initial error /rad
% % %     phi_gyro_int_corrected = phi_Gyro_intArd_rad + (Data_exp.phi_roll_comp(1) - phi_Gyro_intArd_rad(1)); % rad/s
% % % 
% % %     phi_gyro_lower_limit = (phi_gyro_int_corrected - error_gyro_curve); % rad/s
% % %     phi_gyro_higher_limit = (phi_gyro_int_corrected + error_gyro_curve); %rad/s
% % %     phi_gyro = (phi_gyro_int_corrected); % overwrite the phi_gyro, as IMU has more higher frequency
% % % else
% % %     phi_gyro_lower_limit = phi_gyro - (error_gyro_curve);
% % %     phi_gyro_higher_limit = phi_gyro + (error_gyro_curve);
% % % end