% sandbox2.m

% Clear the workspace
clear all; close all; clc;
% Tell matlab to look in the src folder for the class files
addpath('src')
cd ..\
% The input file controls the simulation. Easiest thing to do is copy an
% exsiting file and rename it, then change the parameter values to make
% your simulation.
inputfile = 'labCase1.m';
sim = simulation(inputfile);

hfig = sim.showme;