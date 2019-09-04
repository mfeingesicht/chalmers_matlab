function cost = objfunc(XQ)
%% POSITION MEASUREMENT%%
% EXAMPLE 3 - KALMAN FILTER %
%%% Version 10/05/2019 %%
%kalman filter implemented%
%noise implemented with random number not white noise block

t_sim = 100;         %simulation time [s]
v_true = zeros(t_sim,2);

% Definition of a path
lim_1=10;
lim_2=50;
lim_3=70;
lim_4=80;
v_1=3;
v_2=0;
v_3=-2;
v_4=2;
v_5=3;

% Definition of the velocity matrix
for i=0:+1:t_sim
    v_true(i+1,1)=i;
end

for i=0:+1:lim_1
    v_true(i+1,2)=v_1;
end

for i=lim_1:+1:lim_2
    v_true(i+1,2)=v_2;
end

for i=lim_2:+1:lim_3
    v_true(i+1,2)=v_3;
end 

for i=lim_3:+1:lim_4
    v_true(i+1,2)=v_4;
end

for i=lim_4:+1:t_sim
    v_true(i+1,2)=v_5;
end


% Definition of model - state space representation
Ts=0.1;    %sampling time[s]

A=[1 Ts; 0 1];
B=[0;0];
C=[1 0; 0 1];
D=[0;0];

noise_pos_meas=0.51; 
noise_hall=0.01;

% Process noise covariance
% Q = [0.0001 0 ; 0 0.1];
Q = diag(XQ);
% Measurement noise covariance
R = [noise_pos_meas 0; 0 noise_hall];

% Simulation
assignin('base','Q',Q);
sim('model_kalman_190510.slx')

cost = norm(pos_true.Data - pos_filter.Data(:,1));

end