%% FEEDBACK LINEARIZATION OF 2-DOF FLEXIBLE-JOINT MANIPULATOR
% Numerical physical parameters
%
% Computes:
%
% q^(4) = alpha(x) + beta(x)*tau
%
% and feedback linearization law:
%
% tau = beta^{-1}(v-alpha)

clear
clc

%% SYMBOLIC STATE VARIABLES ONLY

syms q1 q2 dq1 dq2 real
syms th1 th2 dth1 dth2 real
syms tau1 tau2 real

%% NUMERICAL PHYSICAL PARAMETERS

% Link masses
m1 = 1.0;
m2 = 1.0;

% Link lengths
l1 = 1.0;
l2 = 1.0;

% Centers of mass
lc1 = 0.5;
lc2 = 0.5;

% Link inertias
I1 = 1/12 * (l1^2 + 3) * m1;
I2 = 1/12 * (l2^2 + 3) * m2;

% Gravity
g = 9.81;

% Joint stiffness
k1 = 2.0;
k2 = 2.0;

% Motor inertias
J1 = 34.7e-7;
J2 = 34.7e-7;

% Motor damping
b1 = 0.0;
b2 = 0.0;

%% STATE VECTORS

q   = [q1; q2];
dq  = [dq1; dq2];

th  = [th1; th2];
dth = [dth1; dth2];

tau = [tau1; tau2];

%% STIFFNESS MATRIX

K = diag([k1 k2]);

%% MOTOR MATRICES

Jm = diag([J1 J2]);
Bm = diag([b1 b2]);

%% INERTIA MATRIX M(q)

M11 = I1 + I2 + m1*lc1^2 + ...
       m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2));

M12 = I2 + m2*(lc2^2 + l1*lc2*cos(q2));

M21 = M12;

M22 = I2 + m2*lc2^2;

M = [M11 M12;
     M21 M22];

%% CORIOLIS MATRIX C(q,dq)

h = -m2*l1*lc2*sin(q2);

C = [h*dq2      h*(dq1+dq2);
    -h*dq1      0];

%% GRAVITY VECTOR

G1 = (m1*lc1 + m2*l1)*g*cos(q1) + ...
      m2*lc2*g*cos(q1+q2);

G2 = m2*lc2*g*cos(q1+q2);

G = [G1; G2];

%% LINK ACCELERATION

qdd = simplify( M \ (K*(th-q) - C*dq - G) );

%% MOTOR ACCELERATION

thetadd = simplify( Jm \ (tau - Bm*dth - K*(th-q)) );

%% STATE VECTOR

x = [q1;
     q2;
     dq1;
     dq2;
     th1;
     th2;
     dth1;
     dth2];

%% STATE DERIVATIVE

dx = [dq1;
      dq2;
      qdd(1);
      qdd(2);
      dth1;
      dth2;
      thetadd(1);
      thetadd(2)];

%% THIRD DERIVATIVE OF q

qddd = simplify( jacobian(qdd, x) * dx );

%% FOURTH DERIVATIVE OF q

qdddd = simplify( jacobian(qddd, x) * dx );

%% EXTRACT beta(x)

beta = simplify( jacobian(qdddd, tau) );

%% EXTRACT alpha(x)

alpha = simplify( subs(qdddd, [tau1 tau2], [0 0]) );

%% DISPLAY RESULTS

disp('====================================')
disp('ALPHA(x)')
disp('====================================')
disp(alpha)

disp('====================================')
disp('BETA(x)')
disp('====================================')
disp(beta)

%% FEEDBACK LINEARIZATION CONTROL

syms v1 v2 real

v = [v1; v2];

tau_FL = simplify( beta \ (v - alpha) );

disp('====================================')
disp('FEEDBACK LINEARIZING CONTROL')
disp('====================================')

disp(tau_FL)

%% EXAMPLE NUMERICAL EVALUATION

% Example state
x_num = [
    0.1;
    0.2;
    0.0;
    0.0;
    0.12;
    0.25;
    0.0;
    0.0
];

% Example virtual input
v_num = [
    1;
    1
];

% Substitute numerical values

alpha_num = double( subs(alpha, x, x_num) );

beta_num = double( subs(beta, x, x_num) );

tau_num = beta_num \ (v_num - alpha_num);

disp('====================================')
disp('NUMERICAL ALPHA')
disp('====================================')
disp(alpha_num)

disp('====================================')
disp('NUMERICAL BETA')
disp('====================================')
disp(beta_num)

disp('====================================')
disp('CONTROL TORQUE tau')
disp('====================================')
disp(tau_num)