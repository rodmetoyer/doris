% symmanip_dualrotor.m
% Symbolic solution of EOMs for coaxial system
clearvars; close all; clc;

% Create mass symbols
syms m1 m2 mv
mt = m1+m2+mv;

% Create rotation matrices
syms fi1 fi2 fi3 sy1 sy2 sy3 theta gamma beta
A_C_O = [cos(beta) sin(beta) 0; -sin(beta) cos(beta) 0; 0 0 1]*...
    [1 0 0; 0 cos(gamma) sin(gamma); 0 -sin(gamma) cos(gamma)]*...
    [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
O_C_A = A_C_O.';
P1_C_A = [cos(fi3) sin(fi3) 0; -sin(fi3) cos(fi3) 0; 0 0 1]*...
    [1 0 0; 0 cos(fi1) sin(fi1); 0 -sin(fi1) cos(fi1)]*...
    [cos(fi2) 0 -sin(fi2); 0 1 0; sin(fi2) 0 cos(fi2)];
A_C_P1 = P1_C_A.';
P1_C_O = P1_C_A*A_C_O;
O_C_P1 = P1_C_O.';
P2_C_A = [cos(sy3) sin(sy3) 0; -sin(sy3) cos(sy3) 0; 0 0 1]*...
    [1 0 0; 0 cos(sy1) sin(sy1); 0 -sin(sy1) cos(sy1)]*...
    [cos(sy2) 0 -sin(sy2); 0 1 0; sin(sy2) 0 cos(sy2)];
A_C_P2 = P2_C_A.';
P2_C_O = P2_C_A*A_C_O;
O_C_P2 = P2_C_O.';

% Create position vectors
syms x1 x2 x3
r_cso_O = [x1; x2; x3];
syms c1 c2 c3
r_cvcs_A = [c1; c2; c3];
syms g1 g2 g3 h1 h2 h3
r_p1cs_A = [g1; g2; g3];
r_p2cs_A = [h1; h2; h3];

% Create velocity vectors
syms u1 u2 u3
Ov_cso_A = [u1; u2; u3];
syms dx1 dx2 dx3
Ov_cso_O = [dx1; dx2; dx3];
syms w1 w2 w3
O_w_A_A = [w1; w2; w3];
syms p3 q3 % Because of how we define the rotor frames, rotation will only ever be in the 3 direction.
A_w_P1_P1 = [0; 0; p3];
A_w_P1_A = A_C_P1*A_w_P1_P1;
O_w_P1_A = O_w_A_A + A_C_P1*A_w_P1_P1;
A_w_P2_P2 = [0; 0; q3];
A_w_P2_A = A_C_P2*A_w_P2_P2;
O_w_P2_A = O_w_A_A + A_C_P2*A_w_P2_P2;

% Create acceleration vectors
% Note that 'a' is used for linear and angular acceleration.
syms du1 du2 du3
Aa_cso_A = [du1; du2; du3]; % Linear acceleration has prepended frame
Oa_cso_A = Aa_cso_A + cross(O_w_A_A,Ov_cso_A);
syms dw1 dw2 dw3
O_a_A_A = [dw1; dw2; dw3];  % Angular accel'n has frame1_a_frame2
syms dp3 dq3 % Because of how we define the rotor frames, rotation will only ever be in the 3 direction.
A_a_P1_P1 = [0; 0; dp3];
A_a_P1_A = A_C_P1*A_a_P1_P1;
A_a_P2_P2 = [0; 0; dq3];
A_a_P2_A = A_C_P2*A_a_P2_P2;

% Create force and moment vectors
syms tcs1 tcs2 tcs3
Tcs_A = [tcs1; tcs2; tcs3];
syms tp11 tp12 tp13 tp21 tp22 tp23
Tp1_A = A_C_P1*[tp11; tp12; tp13]; % Torque about k_P=k_A. Note tp1 and tp2 are the unknown in-plane reaction torques
Tp2_A = A_C_P2*[tp21; tp22; tp23]; % Torque about k_P=k_A. Note tp1 and tp2 are the unknown in-plane reaction torques
syms f1 f2 f3
F_A = [f1; f2; f3];

% Create inertia matrices
% ASSUMPTION: All frames are aligned with their respective principal axes
syms Icv11 Icv22 Icv33
Icv_A = [Icv11,0,0;0,Icv22,0;0,0,Icv33];
temp = makecrossmat(-r_cvcs_A);
Ics_A = Icv_A-mv*temp*temp; % Move vehicle inertia matrix to cs
clear temp;
syms Ip111 Ip122 Ip133 Ip211 Ip222 Ip233
Ip1_P1 = [Ip111,0,0;0,Ip122,0;0,0,Ip133];
Ip1_A = A_C_P1*Ip1_P1*P1_C_A; % Rotate rotor inertia matrix into A frame
Ip2_P2 = [Ip211,0,0;0,Ip222,0;0,0,Ip233];
Ip2_A = A_C_P2*Ip2_P2*P2_C_A; % Rotate rotor inertia matrix into A frame

disp('Doing algebra: torque equations');
% There are 10 terms for the 3 angular system dynamical equations. Four vehicleBody and 6 each rotor.
% VehicleBody
term1 = mv*cross(r_cvcs_A,Oa_cso_A);
term2 = -mv*cross(Ov_cso_A,cross(O_w_A_A,r_cvcs_A));
term3 = Ics_A*O_a_A_A;
term4 = cross(O_w_A_A,(Ics_A*O_w_A_A));
% Rotor1
term5 = m1*cross(cross(O_w_A_A,r_p1cs_A),Ov_cso_A);
term6 = m1*cross(r_p1cs_A,Oa_cso_A);
term7 = m1*cross(r_p1cs_A,cross(O_a_A_A,r_p1cs_A));
term8 = m1*cross(r_p1cs_A,cross(O_w_A_A,cross(O_w_A_A,r_p1cs_A)));
term9 = Ip1_A*(O_a_A_A + cross(O_w_A_A,A_w_P1_A) + A_a_P1_A);
term10 = cross(O_w_P1_A,(Ip1_A*O_w_P1_A));
% Rotor2
term11 = m2*cross(cross(O_w_A_A,r_p2cs_A),Ov_cso_A);
term12 = m2*cross(r_p2cs_A,Oa_cso_A);
term13 = m2*cross(r_p2cs_A,cross(O_a_A_A,r_p2cs_A));
term14 = m2*cross(r_p2cs_A,cross(O_w_A_A,cross(O_w_A_A,r_p2cs_A)));
term15 = Ip2_A*(O_a_A_A + cross(O_w_A_A,A_w_P2_A) + A_a_P2_A);
term16 = cross(O_w_P2_A,(Ip2_A*O_w_P2_A));

% Dynamical Equations | 12 equations and 12 unknowns
rhs = (term1 + term2 + term3 + term4 + term5 + term6 + term7 + term8 + term9 + term10 + term11 + term12 + term13 + term14 + term15 + term16);
rhs = [rhs; mt*Oa_cso_A;term9+term10;term15+term16];
rhs = simplify(rhs);
% Look for patterns of constants in the RHS that we can replace with
% lumped parameters.

lhs = [Tcs_A;F_A;Tp1_A;Tp2_A];
lhs = simplify(lhs);
eqn = rhs == lhs;
% Solve dynamical equations
disp('Solving eqns.');
S = solve(eqn,[dw1,dw2,dw3,du1,du2,du3,tp11,tp12,dp3,tp21,tp22,dq3]);

% Kinematical Equations  | 8 equations and 8 unknowns
dangles = [1/cos(gamma)*(sin(beta)*O_w_A_A(1)+cos(beta)*O_w_A_A(2));...
    1/cos(gamma)*(cos(beta)*cos(gamma)*O_w_A_A(1)-sin(beta)*cos(gamma)*O_w_A_A(2));...
    1/cos(gamma)*(sin(beta)*sin(gamma)*O_w_A_A(1)-cos(beta)*sin(gamma)*O_w_A_A(2)+cos(gamma)*O_w_A_A(3))];
xdot = O_C_A*Ov_cso_A;
% dfi1 and dfi2 are zero since fi1 and fi2 are constant (fi1 and fi2 are always zero in the
% coaxaial case, but even in the general case they are constant and thus
% their derivatives are zero). Same for dsy1 and dsy2
dfi = [0;0;p3];
dsy = [0;0;q3];

% Simplify and send to file
disp('Simplifying S.dw1');
aa1 = simplify(S.dw1); %simplify(expand(S.dw_1));
disp('Simplifying S.dw2');
aa2 = simplify(S.dw2); %simplify(expand(S.dw_2));
disp('Simplifying S.dw3');
aa3 = simplify(S.dw3); %simplify(expand(S.dw_3));
disp('Simplifying S.du1');
aa4 = simplify(S.du1); %simplify(expand(S.du_1));
disp('Simplifying S.du2');
aa5 = simplify(S.du2); %simplify(expand(S.du_2));
disp('Simplifying S.du3');
aa6 = simplify(S.du3); %simplify(expand(S.du_3));
disp('Simplifying S.tp11');
aa7 = simplify(S.tp11); %simplify(expand(xdot(1)));
disp('Simplifying S.tp12');
aa8 = simplify(S.tp12); %simplify(expand(xdot(2)));
disp('Simplifying S.dp3');
aa9 = simplify(S.dp3); %simplify(expand(xdot(3))); 
disp('Simplifying S.tp21');
aa10 = simplify(S.tp21); %simplify(expand(xdot(1)));
disp('Simplifying S.tp22');
aa11 = simplify(S.tp22); %simplify(expand(xdot(2)));
disp('Simplifying S.dq3');
aa12 = simplify(S.dq3); %simplify(expand(xdot(3)));
disp('Simplifying kinematical equations');
aa13 = simplify(dangles(1));
aa14 = simplify(dangles(2));
aa15 = simplify(dangles(3));
aa16 = simplify(xdot(1));
aa17 = simplify(xdot(2));
aa18 = simplify(xdot(3));
aa19 = dfi(3);
aa20 = dsy(3);

disp('Sending to file');  
fid = fopen('equations_minassumptions_dual.txt','w');
fprintf(fid,'%s %s\n','dw1 = ', aa1);
fprintf(fid,'%s %s\n','dw2 = ', aa2);
fprintf(fid,'%s %s\n','dw3 = ', aa3);
fprintf(fid,'%s %s\n','du1 = ', aa4);
fprintf(fid,'%s %s\n','du2 = ', aa5);
fprintf(fid,'%s %s\n','du3 = ', aa6);
fprintf(fid,'%s %s\n','tp11 = ', aa7);
fprintf(fid,'%s %s\n','tp12 = ', aa8);
fprintf(fid,'%s %s\n','dp3 = ', aa9);
fprintf(fid,'%s %s\n','tp21 = ', aa10);
fprintf(fid,'%s %s\n','tp22 = ', aa11);
fprintf(fid,'%s %s\n','dq3 = ', aa12);
fprintf(fid,'%s %s\n','dtheta = ', aa13);
fprintf(fid,'%s %s\n','dgamma = ', aa14);
fprintf(fid,'%s %s\n','dbeta = ', aa15);
fprintf(fid,'%s %s\n','dx1 = ', aa16);
fprintf(fid,'%s %s\n','dx2 = ', aa17);
fprintf(fid,'%s %s\n','dx3 = ', aa18);
fprintf(fid,'%s %s\n','dfi3 = ', aa19);
fprintf(fid,'%s %s\n','dsy3 = ', aa20);

fclose(fid);

fprintf('\nDone\n');

function mat = makecrossmat(x)
mat=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end