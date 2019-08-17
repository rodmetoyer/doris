% simmanip.m
% Sympolic manipulation for quick check of EOM math

clearvars; close all; clc;

syms fi th sy t

angs = [fi,th,sy];
sqnc = [1,2,3];

R_xi_x = [1 0           0;...
          0 cos(angs(sqnc(1)))  sin(angs(sqnc(1)));...
          0 -sin(angs(sqnc(1))) cos(angs(sqnc(1)))];
R_xi_y = [cos(angs(sqnc(2))) 0 -sin(angs(sqnc(2)));...
          0          1 0;...
          sin(angs(sqnc(2))) 0 cos(angs(sqnc(2)))];
R_xi_z = [cos(angs(sqnc(3)))  sin(angs(sqnc(3))) 0;...
          -sin(angs(sqnc(3))) cos(angs(sqnc(3))) 0;...
          0          0         1];
        
P_C_A = R_xi_z*R_xi_y*R_xi_x;
A_C_P = P_C_A.';
