% sandbox2.m

% Clear the workspace
clear all; close all; clc;
buhlf = @(a,F,sigma,phi,cn) (1-a)^2*sigma*cn/sin(phi)^2 - (8/9+(4*F-40/9)*a+(50/9-4*F)*a^2);
F = 1;
sigma = 0.1*3/(2*pi*1);
phi = 10*pi/180;
cn = 1.1;
f = @(a) buhlf(a,F,sigma,phi,cn);
a = fzero(f,0.5);

return

syms a F sgma cn phi
lhs1 = 4*a*(1-a)*F;
%lhs2 = 8/9+(4*F-40/9)*a+(50/9-4*F)*a^2;
b1 = 8/9; b2 = (4*F-40/9); b3 = (50/9-4*F); b4 = sgma*cn/sin(phi)^2;
c1 = b1/b4-1; c2 = b2/b4+2; c3 = b3/b4-1;
lhs2 = b1+b2*a+b3*a^2;
sngleqn = c1 + c2*a+c3*a^2;
rhs = (1-a)^2*sgma*cn/sin(phi)^2;
eqn1 = rhs == lhs1;
eqn2 = rhs == lhs2;
eqnsngl = sngleqn == 0;
sln1 = solve(eqn1,a);
sln2 = solve(eqn2,a);
slnsngl = solve(eqnsngl,a);
return
% Tell matlab to look in the src folder for the class files
addpath('src')
cd ..\
% The input file controls the simulation. Easiest thing to do is copy an
% exsiting file and rename it, then change the parameter values to make
% your simulation.
inputfile = 'labCase1.m';
sim = simulation(inputfile);

hfig = sim.showme;