function [A_m B_m] = Lin_Sys(v)

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% 3-DOF Ship model Linearization
% 
% Input:
% v : rear wheel velocity
%
% Output:
% A 3*3 Linearized System Matrix for \dot{x} = A x + B u
% B 3*1 The Coefficient Matrix for input u
% 
% 
% 
% state x:= [ \Phi \delta \dot{\Phi} ]
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

persistent h_real b_real c_real lambda_real a_real g 
% Real Bike Parameters
% Global system (frame+box as 1 volume)
if isempty(h_real)
    h_real = 0.48;             % height of center of mass [m]
    b_real = 1.095;            % length between wheel centers [m]
    c_real = 0.06;             % length between front wheel contact point and the 
    lambda_real = deg2rad(70); % angle of the fork axis [deg]
    a_real = 0.34;             % b_real-0.77;
    g = 9.81;                  % gravity [m/s^2]
end

A_m = [0      0            1;
       0      0            0;
       g/h_real    sin(lambda_real)*(h_real*v^2-g*a_real*c_real)/(h_real^2*b_real)   0];
B_m = [0; 1; (a_real*v*sin(lambda_real))/(h_real*b_real)];
C_m = eye(3);
D_m = zeros(3,1);

end