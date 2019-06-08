% simmanip.m
% Sympolic manipulation for quick check of EOM math

clearvars; close all; clc;

syms xi1 xi2 xi3

xi = [xi1,xi2,xi3];
sqnc = [1,2,3];

R_xi_x = [1 0           0;...
          0 cos(xi(sqnc(1)))  sin(xi(sqnc(1)));...
          0 -sin(xi(sqnc(1))) cos(xi(sqnc(1)))];
R_xi_y = [cos(xi(sqnc(2))) 0 -sin(xi(sqnc(2)));...
          0          1 0;...
          sin(xi(sqnc(2))) 0 cos(xi(sqnc(2)))];
R_xi_z = [cos(xi(sqnc(3)))  sin(xi(sqnc(3))) 0;...
          -sin(xi(sqnc(3))) cos(xi(sqnc(3))) 0;...
          0          0         1];
        
P_C_A = R_xi_z*R_xi_y*R_xi_x;
A_C_P = P_C_A.';
