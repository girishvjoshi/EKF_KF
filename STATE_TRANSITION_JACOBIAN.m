function [PHI] = STATE_TRANSITION_JACOBIAN(x)
global Jr m l  J g dt
global Kt R Kb

A_cont_temp = [0 1 0 0
    -(m*g*l*cos(x(1)))/J -Kb*Kt/(J*R) 0 Kb*Kt/(J*R)
    0 0 0 1
    0 Kb*Kt/(Jr*R) 0 -Kb*Kt/(Jr*R)];

B_cont_temp = [0;-Kt/(J*R);0;Kt/(Jr*R)];

H_cont_temp = [1 0 0 0;-1 0 1 0];

D_cont_temp = 0;

Ts = dt;
ssmodel_temp = ss(A_cont_temp,B_cont_temp,H_cont_temp,D_cont_temp);
ssmodel_discete_temp = c2d(ssmodel_temp, Ts, 'forward');

PHI = ssmodel_discete_temp.a;

       
end