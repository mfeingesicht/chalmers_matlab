clear all
close all
clc

%% Wheel
% Didn't find the exact wheel that is installed on the bike
% A similar looking Shimano wheel with the same dimensions can be found at https://www.rosebikes.se/shimano-2659017-2659017
% Rims have a 622mm diameter and 17mm width
r_rim = 622/2*1e-3; % Interior rim radius [m]
w_rim = 23*1e-3;    % Rim width [m]
h_rim = 24*1e-3;    % Rim height [m]
m_rim = 1;        % Rim weight [kg]

% For inertia computation, the wheel is considered to be a cylindrical tube
% with all the mass in the wall with interior radius r_rim, exterior radius
% r_rim+h_rim, width w_rim and weight m_wheel
J_wheel = (1/12)*m_rim*(3*(r_rim^2 + (r_rim+h_rim)^2) + w_rim^2);


%% Tire
% The tire is a SCHWALBE MARATHON PLUS 700x38c
% https://www.probikeshop.fr/pneu-schwalbe-marathon-plus-smart-guard-700x38c-rigide-11100770/132473.html
r_tire_ext = 700*1e-3; % Tire exterior radius [m]
r_tire_int = 622*1e-3; % Tire interior radius [m]
w_tire = 40*1e-3;      % Tire width [m]
m_tire = 960*1e-3;     % Tire weight [kg]

% For inertia computation, the tire is considered to be a cylindrical tube
% with all the mass in the wall with interior radius r_tire_int, exterior
% radius r_tire_ext, width w_tire and weight m_tire
J_tire = (1/12)*m_tire*(3*(r_tire_int^2 + r_tire_ext^2) + w_tire^2);


%% Brake
% Hub dynamo for roller brake
% https://bike.shimano.com/en-EU/product/component/nexus-c6000-int8/DH-C6000-2R.html
% The brake is considered to be a cylinder of diameter 90mm and width 58mm
r_brake = 9/2*1e-3; % Brake radius [m]
w_brake = 58*1e-3;  % Brake width [m]
m_brake = 960*1e-3; % Brake weight [kg]

J_brake = (1/12)*m_brake*(3*r_brake^2 + w_brake^2);


% Front Hub Roller Brake
% No mass given on Shimano website or datasheet, this part is ignored.


%% Handlebar
% The handlebar is considered to be a straight cylinder
r_handlebar = 32/2*1e-3; % Handlebar radius [m]
w_handlebar = 640*1e-3;  % Handlebar width [m]
m_handlebar = 320*1e-3;  % Handlebar weight [kg]

J_handlebar = (1/12)*m_handlebar*(3*r_handlebar^2 + w_handlebar^2);


%% Total inertia
J_total = J_wheel + J_tire + J_brake + J_handlebar