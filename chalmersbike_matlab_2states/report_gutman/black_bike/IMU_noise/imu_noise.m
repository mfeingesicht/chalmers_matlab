clear all
close all
clc

t = readtable('20200213-142723-SensorTest_Lukas_IMU_lifted.csv');
% t = readtable('20200217-104113-SensorTest_Lukas_IMU_ground.csv');

gx = t.Gx_deg_s_;
t = t.Time_s_;
offset_gx = mean(gx)
noise_gx = gx-offset_gx;

figure;plot(gx);
figure;plot(noise_gx);