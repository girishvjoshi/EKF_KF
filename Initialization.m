global mp Jp mr Jr m lp l lr J g

% Pendulum Parameters
mp = 0.2164; % Mass of Pendulum (kg)
Jp = 2.333e-4; % moment of inertia of the pendulum about its center of mass (Kgm2)
mr = 0.0850; % mass of the wheel
Jr = 2.495e-5; % moment of inertia of the rotor about its center of mass
m  = 0.3014; % combined mass of rotor and pendulum
lp = 0.1173; % distance from pivot to the center of mass of the pendulum
l  = 0.12; % distance from pivot to the center of mass of pendulumand rotor
lr = 0.1270; % distance from pivot to the center of mass of the rotor
J  = 4.572e-3;  % moment of inertia of the combined system
g = 9.81; % Gravitational Constant

% Motor Parameters
global Kt R Kb dt

Kt = 27.4e-3; % Current Constant for Torque(Nm/A)
R  = 12.1; % Armatuer Resistance (ohms)
Kb = 27.4e-3; % Back EMF Constants 

% Continues time State Matrices
global A_cont B_cont H_cont D_cont

A_cont  = [0 1 0 0
           -(m*g*l)/J -Kb*Kt/(J*R) 0 Kb*Kt/(J*R)
           0 0 0 1
           0 Kb*Kt/(Jr*R) 0 -Kb*Kt/(Jr*R)];
       
B_cont = [0;-Kt/(J*R);0;Kt/(Jr*R)];

H_cont = [1 0 0 0;-1 0 1 0];

D_cont = 0;

% Discrete Time State matrices

global A B H D
Ts = dt;
ssmodel = ss(A_cont,B_cont,H_cont,D_cont);
ssmodel_discete = c2d(ssmodel, Ts, 'forward');

A = ssmodel_discete.a;
B = ssmodel_discete.b;
H = ssmodel_discete.c;
D = ssmodel_discete.d;
G = [0;1;0;1];

% Kalman Variables

P_updated = 1*eye(4);
Q_kf = 0.000001;%0.00000001;
Q_ekf = 0.000001;
R_noise = [1.2185e-05 0;0 1.2185e-05];
P_updated_ekf = 1*eye(4);


