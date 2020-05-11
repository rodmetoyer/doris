% checkDistributiveProp.m
clearvars
close all
clc

syms a1 a2 a3 b1 b2 b3 c1 c2 c3 q1 q2 q3
a = [a1; a2; a3];
b = [b1; b2; b3];
c = [c1; c2; c3];
q = [q1; q2; q3];

rhs = cross(a,q) + cross(b,q) + cross(c,q);
lhs = cross(a+b+c,q);
simplify(rhs-lhs)

syms Ip11 Ip22 Ip33 Iq11 Iq22 Iq33
Ip_O = [Ip11,0,0;0,Ip22,0;0,0,Ip33];
syms fi1 fi2 fi3 sy1 sy2 sy3 theta gamma beta
A_C_O = [cos(beta) sin(beta) 0; -sin(beta) cos(beta) 0; 0 0 1]*...
    [1 0 0; 0 cos(gamma) sin(gamma); 0 -sin(gamma) cos(gamma)]*...
    [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
O_C_A = A_C_O.';

rhs = A_C_O*Ip_O;
lhs = Ip_O*O_C_A;