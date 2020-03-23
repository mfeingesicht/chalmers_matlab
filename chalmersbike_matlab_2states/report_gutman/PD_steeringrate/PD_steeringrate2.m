% clear all
% close all
% clc


%% Bike parameters
% Plots
plots = 0 ;

% Sampling time
Ts = 0.01;            % sampling time

% Inertia front wheel
inertia_front = 0.2741;
% inertia_front = 0;

% Forward velocity
% v = 1;                % velocity [m/s]
% v = 2;                % velocity [m/s]
% v = 3;                % velocity [m/s]
v = 4;                % velocity [m/s]
% v = 5;                % velocity [m/s]

% Bike physical parameters
r_wheel = 0.311;      % radius of the wheel
h = 0.2085 + r_wheel; % height of center of mass [m]
b = 1.095;            % length between wheel centers [m]
c = 0.06;             % length between front wheel contact point and the extention of the fork axis [m]
lambda = deg2rad(70); % angle of the fork axis [deg]
a = 0.4964;           % distance from rear wheel to frame's center of mass [m]
IMU_height = 0.45;    % IMU height [m]
g = 9.81;             % gravity [m/s^2]
m = 45;               % Bike mas [kg]
bike_params = [g h b a lambda c m];


%% Bike transfer function
num = [(inertia_front*b)/(m*h) a*v v^2];
den = [b*h 0 -b*g];
sys_tf = tf(num,den);


%% Bike state-space matrices
% Continuous time
[A,B,C,D] = tf2ss(num,den);
sys = ss(A,B,C,D);
sys_fullstates = ss(sys.A,sys.B,eye(size(sys.A)),0);

% Discrete time
sys_dis = c2d(sys,Ts);
A_dis = sys_dis.A;
B_dis = sys_dis.B;
C_dis = sys_dis.C;
D_dis = sys_dis.D;


%% Analysis of the open-loop bike model
disp('Poles of the bike in continuous time');
disp(eig(A));

disp('Poles of the bike in discrete time');
disp(eig(A_dis));

if plots
    figure;bode(sys_tf);
    figure;margin(sys_tf);
end


%% PD phidot feedback
% P_phidot = 1;
% D_phidot = 1;
% A = [0 1 ; (b*g-P_phidot*v^2)/(b*h+a*v*D_phidot) , -(a*v*P_phidot+D_phidot*v^2)/(b*h+a*v*D_phidot)];


%% Analysis of eigenvalues of the closed-loop : fixed Kp, varying Kd
% P_phidot = 1;
% D_phidot_test = (0:0.01:10);
% eigA_test = [];
% for i=1:length(D_phidot_test)
%     A_test = [0 1 ; (b*g-P_phidot*v^2)/(b*h+a*v*D_phidot_test(i)) , -(a*v*P_phidot+D_phidot_test(i)*v^2)/(b*h+a*v*D_phidot_test(i))];
%     eigA_test = [eigA_test eig(A_test)];
% end
% figure;hold on;
% plot(D_phidot_test,real(eigA_test(1,:)));
% plot(D_phidot_test,real(eigA_test(2,:)));
% legend('eig1','eig2');


%% Analysis of eigenvalues of the closed-loop : varying Kp, varying Kd
P_phidot_test = (0:0.05:1);
% D_phidot_test = (0:0.05:1);
D_phidot_test = (0:1:100);
is_stable = [];

s = tf('s');
bike_tf_s = (inertia_front*b*s^2/(m*h) + a*v*s + v^2)/(b*h*s^2 - b*g);
steeringmotor_tf_s = 253.4 / (245.9 + s);

for i=1:length(P_phidot_test)
    for j=1:length(D_phidot_test)        
        % deltadot = -Kp*phidot - Kd*phiddot
        deltadot_pid_s = P_phidot_test(i) + D_phidot_test(j)*s;
%         closedloop_tf = bike_tf_s*steeringmotor_tf_s*deltadot_pid_s/(1+bike_tf_s*steeringmotor_tf_s*deltadot_pid_s);
        closedloop_tf = 1+bike_tf_s*steeringmotor_tf_s*deltadot_pid_s;
        
%         roots_closedloop = pole(closedloop_tf);
        roots_closedloop = zero(closedloop_tf);
        if ~isempty(roots_closedloop)
            is_stable(j,i) = 0.5-0.5*sign(max(real(roots_closedloop)));
        else
            is_stable(j,i) = 0;
        end
    end
end

% figure;hold on;
% surf(P_phidot_test,D_phidot_test,real(eigA1_test),'FaceColor','b');
% surf(P_phidot_test,D_phidot_test,real(eigA2_test),'FaceColor','r');
% xlabel('Kp');
% ylabel('Kd');
% legend('eig1','eig2');
% title('Real part of eigenvalues');
% 
% figure;hold on;
% surf(P_phidot_test,D_phidot_test,imag(eigA1_test),'FaceColor','b');
% surf(P_phidot_test,D_phidot_test,imag(eigA2_test),'FaceColor','r');
% xlabel('Kp');
% ylabel('Kd');
% legend('eig1','eig2');
% title('Imaginary part of eigenvalues');
% 
% figure;hold on;
% surf(P_phidot_test,D_phidot_test,max(real(eigA1_test),real(eigA2_test)));
% xlabel('Kp');
% ylabel('Kd');
% title('Max of Real part of eigenvalues');

figure;hold on;
surf(P_phidot_test,D_phidot_test,is_stable);
xlabel('Kp');
ylabel('Kd');
title('Stable ?');