
% This program Simulates the Reaction Wheel Pendulum
% 04/27/2016
% Author: Girish Joshi
% Term Project: Estimation Theory
% Mechanical and Aerospace Engineering, Oklahoma State University.

clear all
clc

% Simulation Time step
global dt
dt = 0.01;
k = 1;
Final_time = 10;
Time_Count(1) = 1;
%
Initialization;

% Intial Condition
x = [30*pi/180;0;0;0]; % Continues State Initial Condition
X(:,1) = [0*pi/180;0;0;0]; % Discrete State initial Condition
X_ekf(:,1) = x;
Intial_Condition = x;
V = 0;

% Intialize vector for Data logging
Time = [];
States = [];
State_Error = [];

[T X]=ode45(@(t,x) ReactionWheel_Pendulum(t,x),[0:dt:Final_time],Intial_Condition);

for t = 0:dt:Final_time
    
        % Motor Voltage Pulse
    if t >= 5 && t<=6
        
        V= 1;
        
    else
        V = 0;
    end
    
    X_ekf(:,k+1) = Reaction_Wheel_Pendulum(X_ekf(:,k),V);
    
    Time = [Time,k];
    
    k = k+1;
        
end
subplot(1,2,1)
plot([1:1002]*dt,X_ekf,'Linewidth',2)
subplot(1,2,2)
plot(T,X)
