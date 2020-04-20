% symmanip_dualrotor_matrix.m
% Symbolic solution of EOMs for coaxial system
%clearvars; close all; clc;

% Create mass symbols
syms m1 m2 mv
ms = m1+m2+mv;

% Create rotation matrices
syms fi1 fi2 fi3 sy1 sy2 sy3 theta gamma beta
A_C_O = [cos(beta) sin(beta) 0; -sin(beta) cos(beta) 0; 0 0 1]*...
    [1 0 0; 0 cos(gamma) sin(gamma); 0 -sin(gamma) cos(gamma)]*...
    [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
O_C_A = A_C_O.';
P_C_A = [cos(fi3) sin(fi3) 0; -sin(fi3) cos(fi3) 0; 0 0 1]*...
    [1 0 0; 0 cos(fi1) sin(fi1); 0 -sin(fi1) cos(fi1)]*...
    [cos(fi2) 0 -sin(fi2); 0 1 0; sin(fi2) 0 cos(fi2)];
A_C_P = P_C_A.';
P_C_O = P_C_A*A_C_O;
O_C_P = P_C_O.';
Q_C_A = [cos(sy3) sin(sy3) 0; -sin(sy3) cos(sy3) 0; 0 0 1]*...
    [1 0 0; 0 cos(sy1) sin(sy1); 0 -sin(sy1) cos(sy1)]*...
    [cos(sy2) 0 -sin(sy2); 0 1 0; sin(sy2) 0 cos(sy2)];
A_C_Q = Q_C_A.';
Q_C_O = Q_C_A*A_C_O;
O_C_Q = Q_C_O.';

% Create position vectors
syms x1 x2 x3
r_ao_O = [x1; x2; x3];
syms c1 c2 c3 s1 s2 s3
r_cva_A = [c1; c2; c3];
r_csa_A = [s1; s2; s3];
r_csa_A_X = makecrossmat(r_csa_A);
syms g1 g2 g3 h1 h2 h3
r_pa_A = [g1; g2; g3];
r_pa_A_X = makecrossmat(r_pa_A);
r_qa_A = [h1; h2; h3];
r_qa_A_X = makecrossmat(r_qa_A);

% Create velocity vectors
syms u1 u2 u3
Ov_ao_A = [u1; u2; u3];
syms dx1 dx2 dx3
Ov_ao_O = [dx1; dx2; dx3];
syms w1 w2 w3
O_w_A_A = [w1; w2; w3];
O_w_A_P = P_C_A*O_w_A_A;
O_w_A_Q = Q_C_A*O_w_A_A;
syms p3 q3 % Because of how we define the rotor frames, rotation will only ever be in the 3 direction.
A_w_P_P = [0; 0; p3];
A_w_P_A = A_C_P*A_w_P_P;
O_w_P_A = O_w_A_A + A_C_P*A_w_P_P;
O_w_P_P = O_w_A_P + A_w_P_P;
A_w_Q_Q = [0; 0; q3];
A_w_Q_A = A_C_Q*A_w_Q_Q;
O_w_Q_A = O_w_A_A + A_C_Q*A_w_Q_Q;
O_w_Q_Q = O_w_A_Q + A_w_Q_Q;

% Create acceleration vectors
% Note that 'a' is used for linear and angular acceleration.
syms du1 du2 du3
Aa_ao_A = [du1; du2; du3]; % Linear acceleration has prepended frame
Oa_ao_A = Aa_ao_A + cross(O_w_A_A,Ov_ao_A);
syms dw1 dw2 dw3
O_a_A_A = [dw1; dw2; dw3];  % Angular accel'n has frame1_a_frame2
syms dp3 dq3 % Because of how we define the rotor frames, rotation will only ever be in the 3 direction.
A_a_P_P = [0; 0; dp3];
A_a_P_A = A_C_P*A_a_P_P;
A_a_Q_Q = [0; 0; dq3];
A_a_Q_A = A_C_Q*A_a_Q_Q;

% Create inertia matrices
% ASSUMPTION: All frames are aligned with their respective principal axes
syms Icv11 Icv22 Icv33
Icv_A = [Icv11,0,0;0,Icv22,0;0,0,Icv33];
temp = makecrossmat(-r_cva_A);
Ia_A = Icv_A-mv*temp*temp; % Move vehicle inertia matrix to point A
clear temp;
syms Ip11 Ip22 Ip33 Iq11 Iq22 Iq33
Ip_P = [Ip11,0,0;0,Ip22,0;0,0,Ip33];
Ip_A = A_C_P*Ip_P*P_C_A; % Rotate rotor inertia matrix into A frame
Iq_Q = [Iq11,0,0;0,Iq22,0;0,0,Iq33];
Iq_A = A_C_Q*Iq_Q*Q_C_A; % Rotate rotor inertia matrix into A frame

disp('Building c vector');
cvec1 = ms*cross(O_w_A_A,Ov_ao_A) + ms*cross(O_w_A_A,cross(O_w_A_A,r_csa_A));
cvec2 = cross(O_w_A_A,Ia_A*O_w_A_A) - mv*cross(O_w_A_A,cross(Ov_ao_A,r_cva_A)) ...
    - m1*cross(O_w_A_A,cross(Ov_ao_A,r_pa_A)) + m1*cross(r_pa_A,cross(O_w_A_A,cross(O_w_A_A,r_pa_A)))...
    + Ip_A*cross(O_w_A_A,A_w_P_A) + cross(O_w_P_A,Ip_A*O_w_P_A)...
    - m2*cross(O_w_A_A,cross(Ov_ao_A,r_qa_A)) + m2*cross(r_qa_A,cross(O_w_A_A,cross(O_w_A_A,r_qa_A)))...
    + Iq_A*cross(O_w_A_A,A_w_Q_A) + cross(O_w_Q_A,Iq_A*O_w_Q_A);
cvec3 = Ip_P*cross(O_w_A_P,A_w_P_P) + cross(O_w_P_P,Ip_P*O_w_P_P);
cvec4 = Ip_P*cross(O_w_A_Q,A_w_Q_Q) + cross(O_w_Q_Q,Iq_Q*O_w_Q_Q);
cvec = [cvec1;cvec2;cvec3;cvec4];

% need to solve for tp1, tp2, tq1, and tq2 to sub in
A = Ia_A - m1*r_pa_A_X*r_pa_A_X - m2*r_qa_A_X*r_qa_A_X + A_C_P*Ip_P*A_C_P + A_C_Q*Iq_Q*A_C_Q;
M = [ms*eye(3) -r_csa_A_X zeros(3) zeros(3);...
    r_csa_A_X A A_C_P*Ip_P A_C_Q*Iq_Q;...
    zeros(3) Ip_P*P_C_A Ip_P zeros(3);...
    zeros(3) Iq_Q*Q_C_A zeros(3) Iq_Q];
beta = [Aa_ao_A; O_a_A_A; A_a_P_P; A_a_Q_Q];
% tau = M*beta + cvec;
% tp1 = tau(7);
% tp2 = tau(8);
% tq1 = tau(10);
% tq2 = tau(11);
% Create force and moment vectors
syms ta1 ta2 ta3
Ta_A = [ta1; ta2; ta3];
syms tp1 tp2 tp3 tq1 tq2 tq3
%syms tp3 tq3
Tp_P = [tp1; tp2; tp3]; % Torque about k_P=k_A. Note tp1 and tp2 are the unknown in-plane reaction torques
Tq_Q = [tq1; tq2; tq3]; % Torque about k_P=k_A. Note tp1 and tp2 are the unknown in-plane reaction torques
syms f1 f2 f3
F_A = [f1; f2; f3];

tauMinusCvec = [F_A;Ta_A;Tp_P;Tq_Q] - cvec;
disp('Simplifying c vector');
tauMinusCvec = simplify(expand(tauMinusCvec));
eqn = beta == transpose(M)*tauMinusCvec;
disp('Solving eqns.');
S = solve(eqn,[dw1,dw2,dw3,du1,du2,du3,tp1,tp2,dp3,tq1,tq2,dq3]);
%S = solve(eqn,[dw1,dw2,dw3,du1,du2,du3,dp3,dq3]);

% Kinematical Equations  | 8 equations and 8 unknowns
dangles = [1/cos(gamma)*(sin(beta)*O_w_A_A(1)+cos(beta)*O_w_A_A(2));...
    1/cos(gamma)*(cos(beta)*cos(gamma)*O_w_A_A(1)-sin(beta)*cos(gamma)*O_w_A_A(2));...
    1/cos(gamma)*(sin(beta)*sin(gamma)*O_w_A_A(1)-cos(beta)*sin(gamma)*O_w_A_A(2)+cos(gamma)*O_w_A_A(3))];
xdot = O_C_A*Ov_ao_A;
% dfi1 and dfi2 are zero since fi1 and fi2 are constant (fi1 and fi2 are always zero in the
% coaxaial case, but even in the general case they are constant and thus
% their derivatives are zero). Same for dsy1 and dsy2
dfi = [0;0;p3];
dsy = [0;0;q3];

% disp('Simplifying S.dw1');
% aa1 = simplify(S.dw1); %simplify(expand(S.dw_1));
% % aa1 = S.dw1; % The dw terms are about half a million characters long
% disp('Simplifying S.dw2');
% aa2 = simplify(S.dw2); %simplify(expand(S.dw_2));
% disp('Simplifying S.dw3');
% aa3 = simplify(S.dw3); %simplify(expand(S.dw_3));
% disp('Simplifying S.du1');
% aa4 = simplify(S.du1); %simplify(expand(S.du_1));
% disp('Simplifying S.du2');
% aa5 = simplify(S.du2); %simplify(expand(S.du_2));
% disp('Simplifying S.du3');
% aa6 = simplify(S.du3); %simplify(expand(S.du_3));

%% Send to file
aa1 = S.dw1; % The dw terms are about half a million characters long
aa2 = S.dw2; %simplify(expand(S.dw_2));
aa3 = S.dw3; %simplify(expand(S.dw_3));
aa4 = S.du1; %simplify(expand(S.du_1));
aa5 = S.du2; %simplify(expand(S.du_2));
aa6 = S.du3; %simplify(expand(S.du_3));
aa7 = S.tp1; %simplify(expand(xdot(1)));
aa8 = S.tp2; %simplify(expand(xdot(2)));
aa9 = S.dp3; %simplify(expand(xdot(3))); 
aa10 = S.tq1; %simplify(expand(xdot(1)));
aa11 = S.tq2; %simplify(expand(xdot(2)));
aa12 = S.dq3; %simplify(expand(xdot(3)));

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
fid = fopen('equations_minassumptions_dual_notsimplified.txt','w');
% fprintf(fid,'%s %s;\n','tauMinusCvec(1) = ', tauMinusCvec(1));
% fprintf(fid,'%s %s;\n','tauMinusCvec(2) = ', tauMinusCvec(2));
% fprintf(fid,'%s %s;\n','tauMinusCvec(3) = ', tauMinusCvec(3));
% fprintf(fid,'%s %s;\n','tauMinusCvec(4) = ', tauMinusCvec(4));
% fprintf(fid,'%s %s;\n','tauMinusCvec(5) = ', tauMinusCvec(5));
% fprintf(fid,'%s %s;\n','tauMinusCvec(6) = ', tauMinusCvec(6));
% fprintf(fid,'%s %s;\n','tauMinusCvec(7) = ', tauMinusCvec(7));
% fprintf(fid,'%s %s;\n','tauMinusCvec(8) = ', tauMinusCvec(8));
% fprintf(fid,'%s %s;\n','tauMinusCvec(9) = ', tauMinusCvec(9));
% fprintf(fid,'%s %s;\n','tauMinusCvec(10) = ', tauMinusCvec(10));
% fprintf(fid,'%s %s;\n','tauMinusCvec(11) = ', tauMinusCvec(11));
% fprintf(fid,'%s %s;\n','tauMinusCvec(12) = ', tauMinusCvec(12));
fprintf(fid,'%s %s;\n','dw1 = ', aa1);
fprintf(fid,'%s %s;\n','dw2 = ', aa2);
fprintf(fid,'%s %s;\n','dw3 = ', aa3);
fprintf(fid,'%s %s;\n','du1 = ', aa4);
fprintf(fid,'%s %s;\n','du2 = ', aa5);
fprintf(fid,'%s %s;\n','du3 = ', aa6);
fprintf(fid,'%s %s;\n','tp1 = ', aa7);
fprintf(fid,'%s %s;\n','tp2 = ', aa8);
fprintf(fid,'%s %s;\n','dp3 = ', aa9);
fprintf(fid,'%s %s;\n','tq1 = ', aa10);
fprintf(fid,'%s %s;\n','tq2 = ', aa11);
fprintf(fid,'%s %s;\n','dq3 = ', aa12);
fprintf(fid,'%s %s;\n','dtheta = ', aa13);
fprintf(fid,'%s %s;\n','dgamma = ', aa14);
fprintf(fid,'%s %s;\n','dbeta = ', aa15);
fprintf(fid,'%s %s;\n','dx1 = ', aa16);
fprintf(fid,'%s %s;\n','dx2 = ', aa17);
fprintf(fid,'%s %s;\n','dx3 = ', aa18);
fprintf(fid,'%s %s;\n','dfi3 = ', aa19);
fprintf(fid,'%s %s;\n','dsy3 = ', aa20);

fclose(fid);

fprintf('\nDone\n');

function mat = makecrossmat(x)
mat=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end