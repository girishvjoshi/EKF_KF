function [x] = Reaction_Wheel_Pendulum(x,V)
   
   global dt
   
    
   xp_actual=Actual_State_Model(x,V);
   
   % 1st step of RK4
   rk1=dt*xp_actual;
   x1=x+rk1/2;
   
   % 2nd step of RK4
   xp_actual=Actual_State_Model(x1,V);
   rk2=dt*xp_actual;
   x1=x+rk2/2;
   
   % 3rd step of RK4
   xp_actual=Actual_State_Model(x1,V);
   rk3=dt*xp_actual;
   x1=x+rk3;
   
   % 4th step of RK4
   xp_actual=Actual_State_Model(x1,V);
   rk4=dt*xp_actual;
   
   x = x+(rk1+2.0*(rk2+rk3)+rk4)/6;
   

%% STATE Model
        
    function [x_dot] = Actual_State_Model(X,e)
        
        global mp Jp mr Jr m lp l lr J g
        global Kt R Kb
        
        theta1_Dot = X(2);
        theta2_Dot = -(m*g*l*sin(X(1)))/J + Kt*Kb*X(4)/(J*R) - Kt*Kb*X(2)/(J*R)-Kt*e/(J*R); 
        
        thetar1_Dot = X(4);
        thetar2_Dot = -Kt*Kb*X(4)/(Jr*R) + Kt*Kb*X(2)/(Jr*R) + Kt*e/(Jr*R); 
        
        x_dot=[theta1_Dot;theta2_Dot;thetar1_Dot;thetar2_Dot];
        
    end

end