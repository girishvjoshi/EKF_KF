function [x_dot] = ReactionWheel_Pendulum(t,x,e_step,step_t1,step_t2)
        global mp Jp mr Jr m lp l lr J g
        global Kt R Kb
        
        if t >= step_t1 && t<=step_t2
            e = e_step;
        else
            e = 0;
        end
        
        theta1_Dot = x(2);
        theta2_Dot = -(m*g*l*sin(x(1)))/J + Kt*Kb*x(4)/(J*R) - Kt*Kb*x(2)/(J*R)-Kt*e/(J*R); 
        
        thetar1_Dot = x(4);
        thetar2_Dot = -Kt*Kb*x(4)/(Jr*R) + Kt*Kb*x(2)/(Jr*R) + Kt*e/(Jr*R); 
        
        x_dot=[theta1_Dot;theta2_Dot;thetar1_Dot;thetar2_Dot];
        
    end