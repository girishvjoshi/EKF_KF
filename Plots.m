figure(1)
subtitle('Kalman');
subplot(2,2,1)
plot(Time,x(:,1)*180/pi,'Linewidth',1.5)
hold on
plot(Time_Count*dt,X(1,:)*180/pi,'k--','Linewidth',1.5);
legend('\theta','\theta_{KF}')
title('Pendulum position','fontsize',15)
set(gca,'fontsize',10)
xlabel('Time(secs)');
ylabel('\theta');
p=mtit('Kalman Filtering',...
	     'fontsize',20,'color',[0 0 1],...
	     'xoff',0.6,'yoff',.05);

subplot(2,2,2)
plot(Time,x(:,2)*180/pi,'Linewidth',1.5);
hold on
plot(Time_Count*dt,X(2,:)*180/pi,'k--','Linewidth',1.5);
legend('\theta','\theta_{dot,KF}')
title('Pendulumn Velocity','fontsize',15);
set(gca,'fontsize',10)
xlabel('Time(secs)');
ylabel('\theta_{dot}');

subplot(2,2,3)
plot(Time,x(:,3)*180/pi,'Linewidth',1.5);
hold on
plot(Time_Count*dt,X(3,:)*180/pi,'k--','Linewidth',1.5);
hold on
plot(Time,Voltage*max(x(:,3))*180/(pi*max(Voltage)),'r--','Linewidth',1.5);
legend('\theta','\theta_{r,KF}','Step Input(V)')
title('Rotor position','fontsize',15)
set(gca,'fontsize',10)
xlabel('Time(secs)');
ylabel('\theta_r');

subplot(2,2,4)
plot(Time,x(:,4)*180/pi,'Linewidth',1.5);
hold on
plot(Time_Count*dt,X(4,:)*180/pi,'k--','Linewidth',1.5);
hold on
plot(Time,Voltage*max(x(:,3))*180/(pi*max(Voltage)),'r--','Linewidth',1.5);
legend('\theta','\theta_{rdot,KF}','Step Input(V)')
title('Rotor Velocity','fontsize',15);
set(gca,'fontsize',10)
xlabel('Time(secs)');
ylabel('\theta_{rdot}');


figure(2)
subplot(2,2,1)
plot(Time,x(:,1)*180/pi,'Linewidth',1.5) 
hold on
plot(Time_Count*dt,X_ekf(1,:)*180/pi,'k--','Linewidth',1.5);
legend('\theta','\theta_{EKF}')
title('Pendulum position','fontsize',15)
set(gca,'fontsize',10)
xlabel('Time(secs)');
ylabel('\theta');

subplot(2,2,2)
plot(Time,x(:,2)*180/pi,'Linewidth',1.5);
hold on;
plot(Time_Count*dt,X_ekf(2,:)*180/pi,'k--','Linewidth',1.5);
legend('\theta','\theta_{dot,EKF}')
title('Pendulumn Velocity','fontsize',15);
set(gca,'fontsize',10)
xlabel('Time(secs)');
ylabel('\theta_{dot}');

subplot(2,2,3)
plot(Time,x(:,3)*180/pi,'Linewidth',1.5);
hold on
plot(Time_Count*dt,X_ekf(3,:)*180/pi,'k--','Linewidth',1.5);
hold on
plot(Time,Voltage*max(x(:,3))*180/(pi*max(Voltage)),'r--','Linewidth',1.5);
legend('\theta','\theta_{r,EKF}','Step Input(V)')
title('Rotor position','fontsize',15)
set(gca,'fontsize',10)
xlabel('Time(secs)');
ylabel('\theta_r');

subplot(2,2,4)
plot(Time,x(:,4)*180/pi,'Linewidth',1.5);
hold on
plot(Time_Count*dt,X_ekf(4,:)*180/pi,'k--','Linewidth',1.5);
hold on
plot(Time,Voltage*max(x(:,3))*180/(pi*max(Voltage)),'r--','Linewidth',1.5);
legend('\theta','\theta_{rdot,EKF}','Step Input(V)')
title('Rotor Velocity','fontsize',15);
set(gca,'fontsize',10)
xlabel('Time(secs)');
ylabel('\theta_{rdot}');
q=mtit('Extended-Kalman Filtering',...
	     'fontsize',20,'color',[0 0 1],...
	     'xoff',0,'yoff',.05);


for i=1:2
set(figure(i),'Position',[50 50 850 650]);
set(figure(i),'PaperOrientation','portrait','PaperSize',[8.5 7],'PaperPositionMode', 'auto', 'PaperType','<custom>');
savestr = ['plot_' num2str(i)];
saveas(figure(i),savestr,'pdf')
end