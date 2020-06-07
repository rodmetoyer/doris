% symmanip_dualrotor_matrix.m
% Symbolic solution of EOMs for coaxial system
% NOTE that the coaxial assumption is embedded in the resultant equations.
% While the dynamic model does not make any assumption about the system
% (other than rigid bodies and axisymmetric rotors), the implementation
% (i.e. simulation model) does assume coaxial.

clear all; close all; clc;

% Create mass symbols
syms m1 m2 mv
ms = m1+m2+mv;

% Create rotation matrices
syms fi1 fi2 fi3 sy1 sy2 sy3 theta gamma beta
A_C_O = [cos(beta) sin(beta) 0; -sin(beta) cos(beta) 0; 0 0 1]*...
    [1 0 0; 0 cos(gamma) sin(gamma); 0 -sin(gamma) cos(gamma)]*...
    [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
O_C_A = A_C_O.';
% P_C_A = [cos(fi3) sin(fi3) 0; -sin(fi3) cos(fi3) 0; 0 0 1]*...
%     [1 0 0; 0 cos(fi1) sin(fi1); 0 -sin(fi1) cos(fi1)]*...
%     [cos(fi2) 0 -sin(fi2); 0 1 0; sin(fi2) 0 cos(fi2)];
% ASSUMPTION!!! Coaxial rotors.
% For the coaxial rotor, fi1 and fi2 are zero.
P_C_A = [cos(fi3) sin(fi3) 0; -sin(fi3) cos(fi3) 0; 0 0 1];
A_C_P = P_C_A.';
P_C_O = P_C_A*A_C_O;
O_C_P = P_C_O.';
% Q_C_A = [cos(sy3) sin(sy3) 0; -sin(sy3) cos(sy3) 0; 0 0 1]*...
%     [1 0 0; 0 cos(sy1) sin(sy1); 0 -sin(sy1) cos(sy1)]*...
%     [cos(sy2) 0 -sin(sy2); 0 1 0; sin(sy2) 0 cos(sy2)];
% For the coaxial rotor, sy1 and sy2 are zero.
Q_C_A = [cos(sy3) sin(sy3) 0; -sin(sy3) cos(sy3) 0; 0 0 1];
A_C_Q = Q_C_A.';
Q_C_O = Q_C_A*A_C_O;
O_C_Q = Q_C_O.';

% Create position vectors
syms x1 x2 x3
r_ao_O = [x1; x2; x3];
% For the coaxial system the center mass of both the vehicle body and the
% system are along the z axis.
% syms c1 c2 c3 s1 s2 s3
syms c1 c3 s1 s3
% r_cva_A = [c1; c2; c3];
% r_csa_A = [s1; s2; s3];
% To reduce the size of the equations, we are restricting the center of
% mass of the vehcile body to be along the k axis and the center of mass of
% the system to be in the i-k plane. The idea here is that you have a
% slender body like a cylinder or spheroid with a ballast placed off of the
% axis to allow for desirable hydrostatic conditions.
r_cva_A = [c1; 0; c3]; % If point A is at the cm of the vehicle body, these are zero.
r_csa_A = [s1; 0; s3];
r_csa_A_X = makecrossmat(r_csa_A);
% For the coaxial system the rotors are on the z axis.
% syms g1 g2 g3 h1 h2 h3
syms g3 h3
% r_pa_A = [g1; g2; g3];
% r_qa_A = [h1; h2; h3];
r_pa_A = [0; 0; g3];
r_qa_A = [0; 0; h3];
r_pa_A_X = makecrossmat(r_pa_A);
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
% Note that 'a' is used for linear and angular acceleration in the naming
% convention.
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
% Icv is the inertia matrix of the vehicle body about the center of mass of
% the vehicle body. This formulation relaxes the assumption that the body
% frame is aligned with the principal axes. The idea, as stated above, is
% that the inertia matrix of the body is the slender body plus the balast
% which may be anywhere in the i-k plane.
syms Icv11 Icv22 Icv33 Icv13
Icv_A = [Icv11,0,-Icv13;0,Icv22,0;-Icv13,0,Icv33];
temp = makecrossmat(-r_cva_A);
Ia_A = Icv_A-mv*temp*temp; % Move vehicle body inertia matrix to point A
clear temp;
% ASSUMPTION: Rotor frames are aligned with their respective principal
% axes. This is easy for 3- or more bladed rotor which act like disks.
% 2-bladed rotors obviously have 3 axes, so the assumption is more for
% them.
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
cvec4 = Iq_Q*cross(O_w_A_Q,A_w_Q_Q) + cross(O_w_Q_Q,Iq_Q*O_w_Q_Q);
cvec = [cvec1;cvec2;cvec3;cvec4];

% now the mass matrix
M11 = ms*eye(3);
M12 = -r_csa_A_X;
M22 = Ia_A - m1*r_pa_A_X*r_pa_A_X - m2*r_qa_A_X*r_qa_A_X + A_C_P*Ip_P*A_C_P + A_C_Q*Iq_Q*A_C_Q;
M23 = A_C_P*Ip_P;
M24 = A_C_Q*Iq_Q;
M33 = Ip_P;
M44 = Iq_Q;
% M = [ms*eye(3) -r_csa_A_X zeros(3) zeros(3);...
%     r_csa_A_X A A_C_P*Ip_P A_C_Q*Iq_Q;...
%     zeros(3) Ip_P*P_C_A Ip_P zeros(3);...
%     zeros(3) Iq_Q*Q_C_A zeros(3) Iq_Q];
M = [M11 M12 zeros(3) zeros(3);...
    transpose(M12) M22 M23 M24;...
    zeros(3) transpose(M23) M33 zeros(3);...
    zeros(3) transpose(M24) zeros(3) M44];
betavec = [Aa_ao_A; O_a_A_A; A_a_P_P; A_a_Q_Q];
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

% make star matrices
xi = [0 0 1];
Mstar = [M11 M12 zeros(3,1) zeros(3,1);...
    transpose(M12) M22 M23*xi.' M24*xi.';...
    zeros(1,3) xi*transpose(M23) xi*M33*xi.' 0;...
    zeros(1,3) xi*transpose(M24) 0 xi*M44*xi.'];
betastar = [Aa_ao_A; O_a_A_A; xi*A_a_P_P; xi*A_a_Q_Q];
taustar = [F_A; Ta_A; xi*Tp_P; xi*Tq_Q];
cvecstar = [cvec1;cvec2;xi*cvec3;xi*cvec4];
tauMinusCvec = taustar - cvecstar;

% tauMinusCvec = [F_A;Ta_A;Tp_P;Tq_Q] - cvec;
% disp('Simplifying c vector');
% tauMinusCvec = simplify(expand(tauMinusCvec));
% eqn = beta == transpose(M)*tauMinusCvec;
% disp('Solving eqns.');
% S = solve(eqn,[dw1,dw2,dw3,du1,du2,du3,tp1,tp2,dp3,tq1,tq2,dq3]);
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
% S = (Mstar.')*tauMinusCvec;
% S = simplify(S);

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
% aa1 = S.dw1; % The dw terms are about half a million characters long
% aa2 = S.dw2; %simplify(expand(S.dw_2));
% aa3 = S.dw3; %simplify(expand(S.dw_3));
% aa4 = S.du1; %simplify(expand(S.du_1));
% aa5 = S.du2; %simplify(expand(S.du_2));
% aa6 = S.du3; %simplify(expand(S.du_3));
% aa7 = S.tp1; %simplify(expand(xdot(1)));
% aa8 = S.tp2; %simplify(expand(xdot(2)));
% aa9 = S.dp3; %simplify(expand(xdot(3))); 
% aa10 = S.tq1; %simplify(expand(xdot(1)));
% aa11 = S.tq2; %simplify(expand(xdot(2)));
% aa12 = S.dq3; %simplify(expand(xdot(3)));

disp('Simplifying kinematical equations');
dangles = simplify(dangles);
xdot = simplify(xdot);
danglesstr = string(dangles);
xdotstr = string(xdot);
danglesstr = strrep(danglesstr,"cos(beta)","cosbeta");
danglesstr = strrep(danglesstr,"sin(beta)","sinbeta");
danglesstr = strrep(danglesstr,"cos(gamma)","cosgamma");
danglesstr = strrep(danglesstr,"sin(gamma)","singamma");
danglesstr = strrep(danglesstr,"cos(theta)","costheta");
danglesstr = strrep(danglesstr,"sin(theta)","sintheta");
danglesstr = strrep(danglesstr,"w1","x(7)");
danglesstr = strrep(danglesstr,"w2","x(8)");
danglesstr = strrep(danglesstr,"w3","x(9)");
xdotstr = strrep(xdotstr,"w1","x(7)");
xdotstr = strrep(xdotstr,"w2","x(8)");
xdotstr = strrep(xdotstr,"w3","x(9)");
xdotstr = strrep(xdotstr,"u1","x(10)");
xdotstr = strrep(xdotstr,"u2","x(11)");
xdotstr = strrep(xdotstr,"u3","x(12)");
xdotstr = strrep(xdotstr,"cos(beta)","cosbeta");
xdotstr = strrep(xdotstr,"sin(beta)","sinbeta");
xdotstr = strrep(xdotstr,"cos(gamma)","cosgamma");
xdotstr = strrep(xdotstr,"sin(gamma)","singamma");
xdotstr = strrep(xdotstr,"cos(theta)","costheta");
xdotstr = strrep(xdotstr,"sin(theta)","sintheta");

aa13 = danglesstr(1);
aa14 = danglesstr(2);
aa15 = danglesstr(3);
aa16 = xdotstr(1);
aa17 = xdotstr(2);
aa18 = xdotstr(3);
aa19 = strrep(string(dfi(3)),'p3',"x(13)");
aa20 = strrep(string(dsy(3)),'q3',"x(15)");

disp('Replacing with states used in vehicleState.m');
cvecstar = string(cvecstar);
cvecstar = strrep(cvecstar,"w1","x(7)");
cvecstar = strrep(cvecstar,"w2","x(8)");
cvecstar = strrep(cvecstar,"w3","x(9)");
cvecstar = strrep(cvecstar,"u1","x(10)");
cvecstar = strrep(cvecstar,"u2","x(11)");
cvecstar = strrep(cvecstar,"u3","x(12)");
cvecstar = strrep(cvecstar,"p3 ","x(13) ");
cvecstar = strrep(cvecstar,"p3*","x(13)*");
cvecstar = strrep(cvecstar,"q3 ","x(15) ");
cvecstar = strrep(cvecstar,"q3*","x(15)*");
cvecstar = strrep(cvecstar,"cos(fi1)","cosfi1");
cvecstar = strrep(cvecstar,"sin(fi1)","sinfi1");
cvecstar = strrep(cvecstar,"cos(sy1)","cossy1");
cvecstar = strrep(cvecstar,"sin(sy1)","sinsy1");
cvecstar = strrep(cvecstar,"cos(fi2)","cosfi2");
cvecstar = strrep(cvecstar,"sin(fi2)","sinfi2");
cvecstar = strrep(cvecstar,"cos(sy2)","cossy2");
cvecstar = strrep(cvecstar,"sin(sy2)","sinsy2");
cvecstar = strrep(cvecstar,"cos(fi3)","cosfi3");
cvecstar = strrep(cvecstar,"sin(fi3)","sinfi3");
cvecstar = strrep(cvecstar,"cos(sy3)","cossy3");
cvecstar = strrep(cvecstar,"sin(sy3)","sinsy3");

disp('Sending to file');  
fid = fopen('equations_coaxAssumGen_matApproach.txt','w');
fprintf(fid,'%s %s;\n','cvecstar(1) = ', cvecstar(1));
fprintf(fid,'%s %s;\n','cvecstar(2) = ', cvecstar(2));
fprintf(fid,'%s %s;\n','cvecstar(3) = ', cvecstar(3));
fprintf(fid,'%s %s;\n','cvecstar(4) = ', cvecstar(4));
fprintf(fid,'%s %s;\n','cvecstar(5) = ', cvecstar(5));
fprintf(fid,'%s %s;\n','cvecstar(6) = ', cvecstar(6));
fprintf(fid,'%s %s;\n','cvecstar(7) = ', cvecstar(7));
fprintf(fid,'%s %s;\n','cvecstar(8) = ', cvecstar(8));

% fprintf(fid,'%s %s;\n',char(betastar(1)), char(S(1)));
% fprintf(fid,'%s %s;\n',char(betastar(2)), char(S(2)));
% fprintf(fid,'%s %s;\n',char(betastar(3)), char(S(3)));
% fprintf(fid,'%s %s;\n',char(betastar(4)), char(S(4)));
% fprintf(fid,'%s %s;\n',char(betastar(5)), char(S(5)));
% fprintf(fid,'%s %s;\n',char(betastar(6)), char(S(6)));
% fprintf(fid,'%s %s;\n',char(betastar(7)), char(S(7)));
% fprintf(fid,'%s %s;\n',char(betastar(8)), char(S(8)));


% fprintf(fid,'%s %s;\n','dw1 = ', aa1);
% fprintf(fid,'%s %s;\n','dw2 = ', aa2);
% fprintf(fid,'%s %s;\n','dw3 = ', aa3);
% fprintf(fid,'%s %s;\n','du1 = ', aa4);
% fprintf(fid,'%s %s;\n','du2 = ', aa5);
% fprintf(fid,'%s %s;\n','du3 = ', aa6);
% fprintf(fid,'%s %s;\n','tp1 = ', aa7);
% fprintf(fid,'%s %s;\n','tp2 = ', aa8);
% fprintf(fid,'%s %s;\n','dp3 = ', aa9);
% fprintf(fid,'%s %s;\n','tq1 = ', aa10);
% fprintf(fid,'%s %s;\n','tq2 = ', aa11);
% fprintf(fid,'%s %s;\n','dq3 = ', aa12);
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