% simmanip2.m
% Sympolic manipulation for quick check of EOM math

clearvars; close all; clc;
syms fi1(t) fi2(t) fi3(t) p1 p2 p3 df1 df2 df3
P1_C_A = [cos(fi3) sin(fi3) 0; -sin(fi3) cos(fi3) 0; 0 0 1]*...
    [1 0 0; 0 cos(fi1) sin(fi1); 0 -sin(fi1) cos(fi1)]*...
    [cos(fi2) 0 -sin(fi2); 0 1 0; sin(fi2) 0 cos(fi2)];
A_C_P1 = P1_C_A.';

omgX = [0 -p3 p2; p3 0 -p1; -p2 p1 0];

A_C_P1_dot = diff(A_C_P1,t);
A_C_P1_dot = subs(A_C_P1_dot,{diff(fi1(t), t),diff(fi2(t), t),diff(fi3(t), t)},{sym('df1'),sym('df2'),sym('df3')});
eqn = omgX == P1_C_A*A_C_P1_dot;
S = solve(eqn,[df1,df2,df3]);