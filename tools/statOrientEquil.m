% statOrientEquil.m

%clearvars; close all; clc;

a1 = 0.0051063829787234;
a3 = 0.0191489361702128;
sgma = 0.97;
theta = 0:0.01:2*pi;
mom = 0.5*sin(theta)+a3*sin(theta)+a1*cos(theta)-0.5/sgma*sin(theta);

figure
plot(theta*180/pi,mom)
xline(90);
xline(270);
yline(0);