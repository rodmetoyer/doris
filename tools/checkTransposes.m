% checkTranposes.m

clearvars; close all; clc;

A = sym('A%d%d',[3 3]);
%B = sym('B%d%d',[3 3]);
B = transpose(A);
c = sym('c%d',[3 1]);

xi = [0 0 1];
M32s = xi*B;
M23s = A*transpose(xi);
reslt = M32s - transpose(M23s)