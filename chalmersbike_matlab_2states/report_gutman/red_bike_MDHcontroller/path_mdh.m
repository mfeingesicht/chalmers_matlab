clear all
close all
clc

Ts = 0.1;
total_time = 30;
time_array = 0:Ts:total_time;

slope = 0.01;
v = 14/3.6;

heading = 0*time_array;
heading(2:16) = atan(slope);
heading(101:115) = -atan(slope);

path_x = cumtrapz(time_array,v*cos(heading));
path_y = cumtrapz(time_array,v*sin(heading));
figure;plot((180/pi)*heading);
figure;plot(path_x,path_y);

dlmwrite('heading_mdh.csv',heading');
dlmwrite('path_mdh.csv',[heading' path_x' path_y']);