% testMovieMaker.m
% script to test the simulation class makeMovie method

clearvars; close all; clc;

addpath('..\src');
% We should be working from the doris folder, so switch over.
cd ..\

simname = 'dualRotorBaseline';
simulation.makeMovie(simname);

% switch back to test folder
cd test