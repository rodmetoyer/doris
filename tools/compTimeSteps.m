% compTimeSteps

clearvars; close all; clc;
addpath('..\src');
cd ..\ % Working from the top folder

BL2 = simulation.loadsim('BL2case6.m');
XL2 = simulation.loadsim('XL2case6.m');
difBX = XL2.states - BL2.states;

figure
plot(difBX)

cd tools