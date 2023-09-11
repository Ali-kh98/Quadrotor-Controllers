close all
t1=linspace(0,250,length(error_smc(:,1)));
t2=linspace(0,250,length(error_ib(:,1)));
%% trajectory tracking
figure
plot3(state_smc(:,1)',state_smc(:,2)',state_smc(:,3)','r')
hold on;xlabel('X');ylabel('Y');zlabel('Z');
plot3(state_ib(:,1)',state_ib(:,2)',state_ib(:,3)')
grid on;legend('IB-SMC','IB');
title('Helix trajectory tracking under external disturbances')
%% trajectory error
figure
subplot(3,1,1)
plot(t1,error_smc(:,1)','r')
title('Position errors under external disturbances')
xlabel('time(s)');ylabel('e_x');hold on;
plot(t2,error_ib(:,1)')
legend('IB-SMC','IB');grid on
hold on
r = 9;xc = 59;yc = 0;
theta = linspace(0,2*pi);
x = r*cos(theta) + xc;
y = 0.45*r*sin(theta) + yc;
plot(x,y,x+100,y)

signal=error_smc(:,1)';
signal2=error_ib(:,1)';
axes('position',[0.36 0.74 0.1 0.1])
box on
indexOfInterest = (t1 < 65) & (t1 > 50);
indexOfInterest2 = (t2 < 65) & (t2 > 50); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight
signal=error_smc(:,1)';
signal2=error_ib(:,1)';
axes('position',[0.68 0.74 0.1 0.1])
box on
indexOfInterest = (t1 < 165) & (t1 > 150);
indexOfInterest2 = (t2 < 165) & (t2 > 150); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight

subplot(3,1,2)
plot(t1,error_smc(:,2)','r')
xlabel('time(s)');ylabel('e_y');hold on;
plot(t2,error_ib(:,2)')
legend('IB-SMC','IB');grid on
hold on
r =3;xc = 58;yc = 0;
theta = linspace(0,2*pi);
x = 2.5*r*cos(theta) + xc;
y = 0.6*r*sin(theta) + yc;
plot(x,y,x+100,y)

signal=error_smc(:,2)';
signal2=error_ib(:,2)';
axes('position',[0.36 0.45 0.1 0.1])
box on
indexOfInterest = (t1 < 65) & (t1 > 50);
indexOfInterest2 = (t2 < 65) & (t2 > 50); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight
signal=error_smc(:,2)';
signal2=error_ib(:,2)';
axes('position',[0.68 0.45 0.1 0.1])
box on
indexOfInterest = (t1 < 165) & (t1 > 150);
indexOfInterest2 = (t2 < 165) & (t2 > 150); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight

subplot(3,1,3)
plot(t1,error_smc(:,3)','r')
xlabel('time(s)');ylabel('e_z');hold on;
plot(t2,error_ib(:,3)')
legend('IB-SMC','IB');grid on;
hold on
r =1;xc = 59;yc = 0;
theta = linspace(0,2*pi);
x = 12*r*cos(theta) + xc;
y = 0.5*r*sin(theta) + yc;
plot(x,y,x+100,y)

signal=error_smc(:,3)';
signal2=error_ib(:,3)';
axes('position',[0.36 0.12 0.1 0.1])
box on
indexOfInterest = (t1 < 65) & (t1 > 50);
indexOfInterest2 = (t2 < 65) & (t2 > 50); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight
signal=error_smc(:,3)';
signal2=error_ib(:,3)';
axes('position',[0.68 0.12 0.1 0.1])
box on
indexOfInterest = (t1 < 165) & (t1 > 150);
indexOfInterest2 = (t2 < 165) & (t2 > 150); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight
%% energy consumption
figure
plot(t1,energy_smc','r')
hold on;xlabel('time(s)');ylabel('Energy Consumption')
plot(t2,energy_ib')
grid on;legend('IB-SMC','IB');
title('Energy consumption of two controllers')
%% desired angles
figure
subplot(2,1,1)
plot(t1,phi_d_smc','r')
title('Attitude commands under external disturbances')
xlabel('time(s)');ylabel('phi_d');hold on;
plot(t2,phi_d_ib')
legend('IB-SMC','IB');grid on;
hold on
r = 0.2;xc = 59;yc = 0.02;
theta = linspace(0,2*pi);
x = 80*r*cos(theta) + xc;
y = 0.2*r*sin(theta) + yc;
plot(x,y)
hold on
plot(x+100,y-0.02)

signal=phi_d_smc';
signal2=phi_d_ib';
axes('position',[0.37 0.63 0.1 0.1])
box on
indexOfInterest = (t1 < 65) & (t1 > 50);
indexOfInterest2 = (t2 < 65) & (t2 > 50); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2))
axis tight
signal=phi_d_smc';
signal2=phi_d_ib';
axes('position',[0.71 0.63 0.1 0.1])
box on
indexOfInterest = (t1 < 165) & (t1 > 150);
indexOfInterest2 = (t2 < 165) & (t2 > 150); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2))
axis tight

subplot(2,1,2)
plot(t1,theta_d_smc','r')
xlabel('time(s)');ylabel('theta_d');hold on;
plot(t2,theta_d_ib')
legend('IB-SMC','IB');grid on

hold on
r = 0.3;xc = 56;yc =0;
theta = linspace(0,2*pi);
x = 25*r*cos(theta) + xc;
y = 1.2*r*sin(theta) + yc;
plot(x,y)
hold on
plot(x+100,y)

signal=theta_d_smc';
signal2=theta_d_ib';
axes('position',[0.36 0.3 0.1 0.1])
box on
indexOfInterest = (t1 < 65) & (t1 > 50);
indexOfInterest2 = (t2 < 65) & (t2 > 50); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2))
axis tight
signal=theta_d_smc';
signal2=theta_d_ib';
axes('position',[0.7 0.3 0.1 0.1])
box on
indexOfInterest = (t1 < 165) & (t1 > 150);
indexOfInterest2 = (t2 < 165) & (t2 > 150); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2))
axis tight
%% Velocity
figure
subplot(3,1,1)
plot(t1,velocity_smc(:,1)','r')
title('Velocity under external disturbances')
xlabel('time(s)');ylabel('v_x');hold on;
plot(t2,velocity_ib(:,1)')
legend('IB-SMC','IB');grid on
hold on
r = 9;xc = 59;yc = 0;
theta = linspace(0,2*pi);
x = r*cos(theta) + xc;
y = 0.45*r*sin(theta) + yc;
plot(x,y,x+100,y)

signal=velocity_smc(:,1)';
signal2=velocity_ib(:,1)';
axes('position',[0.36 0.82 0.1 0.1])
box on
indexOfInterest = (t1 < 65) & (t1 > 50);
indexOfInterest2 = (t2 < 65) & (t2 > 50); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight
signal=velocity_smc(:,1)';
signal2=velocity_ib(:,1)';
axes('position',[0.68 0.82 0.1 0.1])
box on
indexOfInterest = (t1 < 165) & (t1 > 150);
indexOfInterest2 = (t2 < 165) & (t2 > 150); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight

subplot(3,1,2)
plot(t1,velocity_smc(:,2)','r')
xlabel('time(s)');ylabel('v_y');hold on;
plot(t2,velocity_ib(:,2)')
legend('IB-SMC','IB');grid on
hold on
r =3;xc = 58;yc = 0;
theta = linspace(0,2*pi);
x = 2.5*r*cos(theta) + xc;
y = 0.6*r*sin(theta) + yc;
plot(x,y,x+100,y)

signal=velocity_smc(:,2)';
signal2=velocity_ib(:,2)';
axes('position',[0.36 0.52 0.1 0.1])
box on
indexOfInterest = (t1 < 65) & (t1 > 50);
indexOfInterest2 = (t2 < 65) & (t2 > 50); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight
signal=velocity_smc(:,2)';
signal2=velocity_ib(:,2)';
axes('position',[0.68 0.52 0.1 0.1])
box on
indexOfInterest = (t1 < 165) & (t1 > 150);
indexOfInterest2 = (t2 < 165) & (t2 > 150); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight

subplot(3,1,3)
plot(t1,velocity_smc(:,3)','r')
xlabel('time(s)');ylabel('v_z');hold on;
plot(t2,velocity_ib(:,3)')
legend('IB-SMC','IB');grid on;
hold on
r =1;xc = 59;yc = 0.3;
theta = linspace(0,2*pi);
x = 12*r*cos(theta) + xc;
y = 1*r*sin(theta) + yc;
plot(x,y,x+100,y)

signal=velocity_smc(:,3)';
signal2=velocity_ib(:,3)';
axes('position',[0.36 0.22 0.1 0.1])
box on
indexOfInterest = (t1 < 65) & (t1 > 50);
indexOfInterest2 = (t2 < 65) & (t2 > 50); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight
signal=velocity_smc(:,3)';
signal2=velocity_ib(:,3)';
axes('position',[0.68 0.22 0.1 0.1])
box on
indexOfInterest = (t1 < 165) & (t1 > 150);
indexOfInterest2 = (t2 < 165) & (t2 > 150); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2),'LineWidth',3)
axis tight
