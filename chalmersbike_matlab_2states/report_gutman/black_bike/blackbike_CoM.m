clear all
close all
clc

l = 1.15;
F1 = 18.2;
F2 = 13.1;
W = 18.2+13.1;
a = F2*l/W,
F4 = 10.5;
h = 0.53;
theta = asin(h/l),
b = ((W*a - F4*l)*cos(theta)) / (W*sin(theta)),