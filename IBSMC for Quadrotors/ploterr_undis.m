close all
t1=linspace(0,25,length(error_smc(:,1)));
t2=linspace(0,25,length(error_ib(:,1)));
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
xlabel('time(s)');ylabel('e_x');hold on;
plot(t2,error_ib(:,1)')
legend('IB-SMC','IB');grid on

subplot(3,1,2)
plot(t1,error_smc(:,2)','r')
xlabel('time(s)');ylabel('e_y');hold on;
plot(t2,error_ib(:,2)')
legend('IB-SMC','IB');grid on

subplot(3,1,3)
plot(t1,error_smc(:,3)','r')
xlabel('time(s)');ylabel('e_z');hold on;
plot(t2,error_ib(:,3)')
legend('IB-SMC','IB');grid on;
title('Position errors in three directions')
%% Lift & M
figure
subplot(2,2,1)
plot(t1,lift_smc','r')
hold on;xlabel('time(s)');ylabel('Lift');
plot(t2,lift_ib');grid on;
legend('IB-SMC','IB');

subplot(2,2,2)
plot(t1,M_smc(:,1)','r')
hold on;xlabel('time(s)');ylabel('M_x');
plot(t2,M_ib(:,1)');grid on;
legend('IB-SMC','IB');

hold on
r = 0.6;xc = 17;yc = 0;
theta = linspace(0,2*pi);
x = 4*r*cos(theta) + xc;
y = 0.01*r*sin(theta) + yc;
plot(x,y)

signal=M_smc(:,1)';
signal2=M_ib(:,1)';
axes('position',[0.66 0.74 0.1 0.1])
box on
indexOfInterest = (t1 < 18) & (t1 >14);
indexOfInterest2 = (t2 < 18) & (t2 > 14); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2))
axis tight



subplot(2,2,3)
plot(t1,M_smc(:,2)','r')
hold on;xlabel('time(s)');ylabel('M_y');
plot(t2,M_ib(:,2)');grid on;
legend('IB-SMC','IB');

hold on
r = 0.2;xc = 16;yc = 0;
theta = linspace(0,2*pi);
x = 10*r*cos(theta) + xc;
y = 0.01*r*sin(theta) + yc;
plot(x,y)

signal=M_smc(:,2)';
signal2=M_ib(:,2)';
axes('position',[0.25 0.3 0.1 0.1])
box on
indexOfInterest = (t1 < 18) & (t1 >14);
indexOfInterest2 = (t2 < 18) & (t2 > 14); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2))
axis tight

subplot(2,2,4)
plot(t1,M_smc(:,3)','r')
hold on;xlabel('time(s)');ylabel('M_z');
plot(t2,M_ib(:,3)');grid on;
legend('IB-SMC','IB');
hold on
r = 1.5;xc = 16;yc = 0;
theta = linspace(0,2*pi);
x = 1*r*cos(theta) + xc;
y = 2.5*r*sin(theta) + yc;
plot(x,y)

signal=M_smc(:,3)';
signal2=M_ib(:,3)';
axes('position',[0.66 0.3 0.1 0.1])
box on
indexOfInterest = (t1 < 18) & (t1 >14);
indexOfInterest2 = (t2 < 18) & (t2 > 14); 
plot(t1(indexOfInterest),signal(indexOfInterest),'r',t2(indexOfInterest2),signal2(indexOfInterest2))
axis tight
%% energy consumption
figure
plot(t1,energy_smc','r')
hold on;;grid on;
plot(t2,energy_ib')
legend('IB-SMC','IB');
xlabel('time(s)');ylabel('Energy Consumption');
