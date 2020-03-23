% setup

a=0.7;
b=1.5;
h=0.6;

% a=0.4964;
% b=1.0950;
% h=0.5195;

v=3;
g=9.81;
Ts=0.04;
Td=1; % tim delay
A=[0 g/h;1 0];
B=(a*v/(b*h))*[1; v/a];
C=[1 0;0 1];

% A=[0 g/h;1 0];
% B=[a*v/(b*h); v^2/(b*h)];
% C=[0 1];

R=1;
Q=[10 0;0 10];
[Kd,S,e] = lqrd(A,B,Q,R,Ts);

% poles
eig(A-B*Kd)