close all
clear all
clc
% model 2nd and 3rd motors only (in this code subscript 1 corresponds to motor/link 2, 2 to 3)
H_num = (80+47+45)/1000; % from base to center of motor 2
g_num = 9.81;

r1_num = 38.1/2/1000; % link 2 radius
r2_num = 25/2/1000; % link 3 radius (heavily simplified everything from motor 3 onwards as cylinders)

R1_num = 44/1000; % motor 2 radius
R2_num = 45/1000; % motor 3 radius

l1_num = 34/1000; % link 2 length
l2_num = 1; % link 3 length (heavily simplified everything from motor 3 onwards as cylinders)

L1_num = 77/1000; % motor 2 length (height)
L2_num = 64/1000; % motor 3 length (height)

m1_num = 0.2; % link 2 mass
m2_num = 1.4 + 1.4 + 0.72 + 0.19 + 0.11 + 0.13 + 0.3; % link 3 mass (heavily simplified everything from motor 3 onwards as cylinders)

M1_num = 2.685; % motor 2 mass
M2_num = 1.4; % motor 3 mass

syms t q1(t) q2(t) 

% rotation matrices and frames velocity
R01 = [cos(q1) 0 sin(q1);
    0 1 0;
    -sin(q1) 0 cos(q1)];
R10 = R01.';
R12 = [cos(q2) 0 sin(q2);
    0 1 0;
    -sin(q2) 0 cos(q2)];
R21 = R12.';

w1_1 = [0;diff(q1,t);0];

w21_2 = [0;diff(q2,t);0];
w2_2 = w21_2 + R21*w1_1;

syms M1 m1 M2 m2 
syms R1 r1 R2 r2 
syms L1 l1 L2 l2 
syms H g

% Potential energy
V_M1 = M1*g*H;
V_L1 = m1*g*(H+((l1-R1-R2)/2+R1)*cos(q1));
V_M2 = M2*g*(H+l1*cos(q1));
V_L2 = m2*g*(H+cos(q1)*(l1+cos(q2)*l2));

V = V_M2 + V_L2 + V_L1 + V_M1;

% Position vectors
rOG1_0 = [0;0;H];

rG1D1_1 = [0;0;l1/2];
rOD1_0 = rOG1_0 + R01*rG1D1_1;

rG1G2_1 = [0;0;l1];
rOG2_0 = rOG1_0 + R01*rG1G2_1;

rG2P_2 = [0;0;l2];
rOD2_0 = rOG2_0 + R01*R12*rG2P_2;

rOG1_dot_0 = diff(rOG1_0,t);
rOD1_dot_0 = diff(rOD1_0,t);
rOG2_dot_0 = diff(rOG2_0,t);
rOD2_dot_0 = diff(rOD2_0,t);

% Inertia tensors
I_M1_G1_1 = [M1/12*(3*R1^2+L1^2) 0 0;
    0 M1/2*R1^2 0;
    0 0 M1/12*(3*R1^2+L1^2)];

I_L1_D1_1 = [m1/12*(3*m1^2+l1^2) 0 0;
    0 m1/12*(3*r1^2+l1^2) 0;
    0 0 m1/2*r1^2];

I_M2_G2_2 = [M2/12*(3*R2^2+L2^2) 0 0;
    0 M2/2*R2^2 0;
    0 0 M2/12*(3*R2^2+L2^2)];

I_L2_D2_2 = [m2/12*(3*m2^2+l2^2) 0 0;
    0 m2/12*(3*r2^2+l2^2) 0;
    0 0 m2/2*r2^2];

% Kinetic energy
T_M1 = 1/2*M1*(rOG1_dot_0.')*rOG1_dot_0 + 1/2*(w1_1.')*I_M1_G1_1*w1_1;
T_L1 = 1/2*m1*(rOD1_dot_0.')*rOD1_dot_0 + 1/2*(w1_1.')*I_L1_D1_1*w1_1;
T_M2 = 1/2*M2*(rOG2_dot_0.')*rOG2_dot_0 + 1/2*(w2_2.')*I_M2_G2_2*w2_2;
T_L2 = 1/2*m2*(rOD2_dot_0.')*rOD2_dot_0 + 1/2*(w2_2.')*I_L2_D2_2*w2_2;

T = T_L2 + T_M2 + T_L1 + T_M1;

% Lagrangian
L = T - V;

% Dissipitation function
D1_num = 0.7;
D2_num = 0.7;

R = 1/2*(D1_num*diff(q1,t)^2 + D2_num*diff(q2,t)^2);

% Generalised forces
syms t T1(t) T2(t)
M1_1 = [0;T1;0];
M2_2 = [0;T2;0];

Q1 = (M1_1.')*diff(w1_1,diff(q1,t)) + (M2_2.')*diff(w2_2,diff(q1,t));
Q2 = (M1_1.')*diff(w1_1,diff(q2,t)) + (M2_2.')*diff(w2_2,diff(q2,t));

% symbolic EoMs
eq1_lhs = diff(diff(L,diff(q1,t)),t) - diff(L,q1) + diff(R,diff(q1,t));
eq2_lhs = diff(diff(L,diff(q2,t)),t) - diff(L,q2) + diff(R,diff(q2,t));

eq1 = eq1_lhs-Q1 == 0;
eq2 = eq2_lhs-Q2 == 0;

syms q1_ddot q2_ddot q1_dot q2_dot % subs in symbolic vars to perform differential
eom1 = subs(eq1, [diff(diff(q1,t),t) diff(diff(q2,t),t) diff(q1,t) diff(q2,t)], ...
    [q1_ddot q2_ddot q1_dot q2_dot]);
eom2 = subs(eq2, [diff(diff(q1,t),t) diff(diff(q2,t),t) diff(q1,t) diff(q2,t)], ...
    [q1_ddot q2_ddot q1_dot q2_dot]);

% subs in the numeric value
eom1_num = subs(eom1,[m1 m2 M1 M2 l1 l2 L1 L2 r1 r2 R1 R2 H g], ...
    [m1_num m2_num M1_num M2_num ...
    l1_num l2_num L1_num L2_num ...
    r1_num r2_num R1_num R2_num ...
    H_num g_num]);
eom2_num = subs(eom2,[m1 m2 M1 M2 l1 l2 L1 L2 r1 r2 R1 R2 H g], ...
    [m1_num m2_num M1_num M2_num ...
    l1_num l2_num L1_num L2_num ...
    r1_num r2_num R1_num R2_num ...
    H_num g_num]);

% These are numeric, non linear EoMs, used for linearization
eom1_num_vpa = simplify(vpa(eom1_num,3)); 
eom2_num_vpa = simplify(vpa(eom2_num,3)); 

eom1_num_rhs = vpa(-lhs(eom1_num_vpa) + rhs(eom1_num_vpa),3); % nonlinear
eom2_num_rhs = vpa(-lhs(eom2_num_vpa) + rhs(eom2_num_vpa),3); % nonlinear

% equilibria
% angular positions at equilibrium are arbitrarily chosen since it is not our interest to control position 
q1_num = pi/4;
q2_num = pi/4;

syms x1 x2 % find input torque at equilibrium
e1 = subs(eom1_num_rhs,[q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], ...
    [0 0 0 0 q1_num q2_num x1 x2]);
e2 = subs(eom2_num_rhs,[q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], ...
    [0 0 0 0 q1_num q2_num x1 x2]);

sol = solve([e1 e2],[x1 x2]);
T1_num = subs(sol.x1,g,g_num);
T2_num = subs(sol.x2,g,g_num);

equilStates = [q1_num q2_num T1_num T2_num];
equilStates = double(equilStates);
save('equilStates.mat','equilStates');

% linearize (ignore motor dynamics so 4 states instead of 6)
df1dq1 = diff(eom1_num_rhs,q1);
df1dq1d = diff(eom1_num_rhs,q1_dot);
df1dq1dd = diff(eom1_num_rhs,q1_ddot);
df1dq2 = diff(eom1_num_rhs,q2);
df1dq2d = diff(eom1_num_rhs,q2_dot);
df1dq2dd = diff(eom1_num_rhs,q2_ddot);

df2dq1 = diff(eom2_num_rhs,q1);
df2dq1d = diff(eom2_num_rhs,q1_dot);
df2dq1dd = diff(eom2_num_rhs,q1_ddot);
df2dq2 = diff(eom2_num_rhs,q2);
df2dq2d = diff(eom2_num_rhs,q2_dot);
df2dq2dd = diff(eom2_num_rhs,q2_ddot);

df1dq1s = double(simplify(subs(df1dq1, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));
df1dq1ds = double(simplify(subs(df1dq1d, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));
df1dq1dds = double(simplify(subs(df1dq1dd, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));
df1dq2s = double(simplify(subs(df1dq2, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));
df1dq2ds = double(simplify(subs(df1dq2d, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));
df1dq2dds = double(simplify(subs(df1dq2dd, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));

df2dq1s = double(simplify(subs(df2dq1, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));
df2dq1ds = double(simplify(subs(df2dq1d, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));
df2dq1dds = double(simplify(subs(df2dq1dd, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));
df2dq2s = double(simplify(subs(df2dq2, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));
df2dq2ds = double(simplify(subs(df2dq2d, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));
df2dq2dds = double(simplify(subs(df2dq2dd, [q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num])));

% linearized eom
f1_lin = df1dq1s*q1 + df1dq1ds*q1_dot + df1dq1dds*q1_ddot + df1dq2s*q2 + df1dq2ds*q2_dot + df1dq2dds*q2_ddot;
f2_lin = df2dq1s*q1 + df2dq1ds*q1_dot + df2dq1dds*q1_ddot + df2dq2s*q2 + df2dq2ds*q2_dot + df2dq2dds*q2_ddot;

F1 = f1_lin == 0;
F2 = f2_lin == 0;

Sol = solve([F1 F2],[q1_ddot q2_ddot]);
f1 = simplify(Sol.q1_ddot);
f2 = simplify(Sol.q2_ddot);

% EoMs in the form of q_ddot = f(q,q_dot)
f1 = vpa(f1,3);
f2 = vpa(f2,3);

% start building state matrix
A3_1 = diff(f1,q1);
A3_2 = diff(f1,q2);
A3_3 = diff(f1,q1_dot);
A3_4 = diff(f1,q2_dot);
A4_1 = diff(f2,q1);
A4_2 = diff(f2,q2);
A4_3 = diff(f2,q1_dot);
A4_4 = diff(f2,q2_dot);

% subs numeric value at equilibrium
A3_1 = subs(A3_1,[q1 q2 T1 T2], ...
    [q1_num q2_num T1_num T2_num]);
A3_2 = subs(A3_2,[q1 q2 T1 T2], ...
    [q1_num q2_num T1_num T2_num]);
A3_3 = subs(A3_3,[q1 q2 T1 T2], ...
    [q1_num q2_num  T1_num T2_num]);
A3_4 = subs(A3_4,[q1 q2 T1 T2], ...
    [q1_num q2_num T1_num T2_num]);
A4_1 = subs(A4_1,[q1 q2 T1 T2], ...
    [q1_num q2_num T1_num T2_num]);
A4_2 = subs(A4_2,[q1 q2 T1 T2], ...
    [q1_num q2_num T1_num T2_num]);
A4_3 = subs(A4_3,[q1 q2 T1 T2], ...
    [q1_num q2_num T1_num T2_num]);
A4_4 = subs(A4_4,[q1 q2 T1 T2], ...
    [q1_num q2_num T1_num T2_num]);

% state matrix
A = [0 0 1 0;
    0 0 0 1;
    A3_1 A3_2 A3_3 A3_4;
    A4_1 A4_2 A4_3 A4_4];
A = simplify(A);
A = vpa(A);
A = double(A);
save('stateMatrix.mat','A');

% input matrix (same procedure as in state matrix)
B3_1 = diff(eom1_num_rhs,T1);
B3_2 = diff(eom1_num_rhs,T2);
B4_1 = diff(eom2_num_rhs,T1);
B4_2 = diff(eom2_num_rhs,T2);

B3_1 = subs(B3_1,[q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num]);
B3_2 = subs(B3_2,[q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num]);
B4_1 = subs(B4_1,[q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num]);
B4_2 = subs(B4_2,[q1_ddot q2_ddot q1_dot q2_dot q1 q2 T1 T2], [0 0 0 0 q1_num q2_num T1_num T2_num]);

B = [0 0;
    0 0;
    B3_1 B3_2;
    B4_1 B4_2];
B = vpa(B(t));
B = double(B);
save('inputMatrix.mat','B');