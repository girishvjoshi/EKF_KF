
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
X(:,1) = mvnrnd([0;0;0;0],sqrt(P_updated));
X_ekf(:,1) = mvnrnd([0;0;0;0],sqrt(P_updated_ekf));
Intial_Condition = x;

% STEP INPUT
V_step = 1;
step_start_time = 5;
step_end_time = 6;

% Intialize vector for Data logging
Time = [];
States = [];
State_Error_kf = [];
State_Error_ekf = [];

[T x] = ode45(@(t,state) ReactionWheel_Pendulum(t,state,V_step,step_start_time,step_end_time),[0:dt:Final_time],Intial_Condition);

for t = 0:dt:Final_time
    
    % Motor Voltage Pulse
    if t >= step_start_time && t<= step_end_time
        
        V= V_step;
        
    else
        V = 0;
    end
    
    %%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
     % PREDICTION STEP
    
     X(:,k+1) = A*X(:,k) + B*V;
        
     % MEASUREMENT VECTOR
     
     Z(:,k+1) = H*x(k,:)' + [random('norm',0,sqrt(R_noise(1,1)));random('norm',0,sqrt(R_noise(2,2)))];
     
     % Error Covariance PREDICTION
     
     P_predictor = A*P_updated*A' + G*Q_kf*G';
     
     % Kalman Gain
     
     Kalman_Gain = P_predictor*H'*inv(H*P_predictor*H' + R_noise);
    
     % Correction Step
    
     X(:,k+1) = X(:,k+1) + Kalman_Gain*(Z(:,k+1) - H*X(:,k+1));
    
    % Error Covariance Propagation 
    
     P_updated = (eye(4) - Kalman_Gain*H)*P_predictor*(eye(4) - Kalman_Gain*H)' + Kalman_Gain*R_noise*Kalman_Gain';
     
     %%
     
     %%%%%%%%% Extended Kalman Filtering %%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
     
     %PREDICTION STEP
     
     X_ekf(:,k+1) = Reaction_Wheel_Pendulum(X_ekf(:,k),V);
     
     % PHI linearized at X(k|k)
     
     PHI = STATE_TRANSITION_JACOBIAN(X_ekf(:,k));
     
     % Error Covariance PREDICTION
     
     P_predictor_ekf = PHI*P_updated_ekf*PHI'+ G*Q_ekf*G';
     
      % Kalman Gain
     
     Kalman_Gain_ekf = P_predictor_ekf*H'*inv(H*P_predictor_ekf*H' + R_noise);
     
     % Correction Step
    
     X_ekf(:,k+1) = X_ekf(:,k+1) + Kalman_Gain_ekf*(Z(:,k+1) - H*X_ekf(:,k+1));
     
     % Error Covariance Propagation 
    
     P_updated_ekf = (eye(4) - Kalman_Gain_ekf*H)*P_predictor_ekf*(eye(4) - Kalman_Gain_ekf*H)'+ Kalman_Gain_ekf*R_noise*Kalman_Gain_ekf';
     
    % STORING DATA FOR PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    P_PRED(:,k) = diag(P_predictor);
    P_UPDATE(:,k) = diag(P_updated);
    P_PRED_ekf(:,k) = diag(P_predictor_ekf);
    P_UPDATE_ekf(:,k) = diag(P_updated_ekf);
    Time = [Time,t];
    Time_Count(k+1) = k+1;
    State_Error_kf = [State_Error_kf,x(k,:)'-X(:,k+1)];
    State_Error_ekf = [State_Error_ekf,x(k,:)'-X_ekf(:,k)];
    Voltage(k) = V;
    k = k+1;
    
end
Plots;