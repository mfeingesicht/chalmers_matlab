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
P_phidot_test = (0:0.1:10);
D_phidot_test = (0:0.1:10);
eigA1_test = [];
eigA2_test = [];

lhs_carac_poly = [0 h 0 -g]; % [phidddot phiddot phidot phi]

for i=1:length(P_phidot_test)
    for j=1:length(D_phidot_test)
%         A_test = [0 1 ; (b*g-P_phidot_test(i)*v^2)/(b*h+a*v*D_phidot_test(j)) , -(a*v*P_phidot_test(i)+D_phidot_test(j)*v^2)/(b*h+a*v*D_phidot_test(j))];
%         eigA_test = eig(A_test);
%         eigA1_test(j,i) = [eigA_test(1)];
%         eigA2_test(j,i) = [eigA_test(2)];
        
        % delta = -Kp*phi - Kd*phidot
        rhs_carac_poly = [(-inertia_front*D_phidot_test(j)/(m*h)) , (-inertia_front*P_phidot_test(i)/(m*h))+(-a*v*D_phidot_test(j))/b , (-a*v*P_phidot_test(i)/b)+(-v^2*D_phidot_test(j)/b) , (-v^2*P_phidot_test(i)/b)];

        carac_poly = lhs_carac_poly - rhs_carac_poly;
        roots_carac = roots(carac_poly);
        eigA1_test(j,i) = [roots_carac(1)];
        eigA2_test(j,i) = [roots_carac(2)];
    end
end

figure;hold on;
surf(P_phidot_test,D_phidot_test,real(eigA1_test),'FaceColor','b');
surf(P_phidot_test,D_phidot_test,real(eigA2_test),'FaceColor','r');
xlabel('Kp');
ylabel('Kd');
legend('eig1','eig2');
title('Real part of eigenvalues');

figure;hold on;
surf(P_phidot_test,D_phidot_test,imag(eigA1_test),'FaceColor','b');
surf(P_phidot_test,D_phidot_test,imag(eigA2_test),'FaceColor','r');
xlabel('Kp');
ylabel('Kd');
legend('eig1','eig2');
title('Imaginary part of eigenvalues');

figure;hold on;
surf(P_phidot_test,D_phidot_test,max(real(eigA1_test),real(eigA2_test)));
xlabel('Kp');
ylabel('Kd');
title('Max of Real part of eigenvalues');

figure;hold on;
surf(P_phidot_test,D_phidot_test,0.5-0.5*sign(max(real(eigA1_test),real(eigA2_test))));
xlabel('Kp');
ylabel('Kd');
title('Stable ?');