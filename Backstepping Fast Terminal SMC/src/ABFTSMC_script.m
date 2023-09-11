clc;clear;close all;close all;

%{ 
    *** Robust Adaptive Backstepping Fast Terminal Sliding Mode Control 
                    RAB-FTSMC ***2022
%}

%%  Load System Parameters
comparison = 1;switcher =1;
if switcher ==1
    % using AB-FTSMC
    beta_phi = 0.2146;
else
    %using AB-SMC
    beta_phi =0;
end

S = @(x) sin(x);
C = @(x) cos(x);

Parameters = Params_Func();

g = Parameters.g;
m = Parameters.m;
Ixx = Parameters.Ixx;
Iyy = Parameters.Iyy;
Izz = Parameters.Izz;
Ir = Parameters.Ir;
k1 = Parameters.k1;
k2 = Parameters.k2;
k3 = Parameters.k3;
k4 = Parameters.k4;
k5 = Parameters.k5;
k6 = Parameters.k6;

kp = Parameters.kp;
cd = Parameters.cd;

a1 = (Iyy-Izz)/Ixx;
a3 = -k1/Ixx;
a4 = (Izz-Ixx)/Iyy;
a6 = -k2/Iyy;
a7 = (Ixx-Iyy)/Izz;
a8 = -k3/Izz;
a9 = -k4/m;
a10 = -k5/m;
a11 = -k6/m;
b1 = 1/Ixx;
b2 = 1/Iyy;
b3 = 1/Izz;

% The Parameters of the Sliding Surfaces
p_phi = 3;
p_theta = p_phi;
p_say = p_phi;

q_phi = 5;
q_theta = q_phi;
q_say = q_phi;

% Constant Coefficients of the Controller
if switcher ==1
    % using AB-FTSMC
    gamma7 = 0.5;
    gamma9 = 1.2;
    gamma11 = 1.5;
    gammaPhi = 75;
    gammaTheta = gammaPhi;
    gammaSay = gammaPhi;
else
    %using AB-SMC
    gamma7 = 0.1;
    gamma9 =gamma7;
    gamma11 = gamma7;
    gammaPhi = 100;
    gammaTheta = gammaPhi;
    gammaSay = gammaPhi;
end

c_phi = 0.01;
c_theta = c_phi;
c_say = c_phi;

alpha_phi = 10;
alpha_theta = alpha_phi;
alpha_say = alpha_phi;

beta_phi = 0.2146;
beta_theta = beta_phi;
beta_say = beta_phi;

% COnstant Coefficients of Position Control
cx1 = 0.8;
cy1 = 4; 
cz1 = 5;

nParams = 6;         
Ics_Params = zeros(nParams,1); 

%% Solver Initialization

SampleTime = 0.01;
tMax = 80;  
t = 0:SampleTime:tMax;

nStates = 12; % Number of States
Size_States = [nStates 1];   
Ics_States = zeros(Size_States);

%%The Initialization of Adaptive Laws
cHat_x2 = zeros(size(t));
cHat_y2 = cHat_x2;
cHat_z2 = cHat_x2;

kHat_phi = cHat_x2;
kHat_theta = cHat_x2;
kHat_say = cHat_x2;

%% Define Reference Trajectories and Their corresponding Time Derivatives

xd = @(t) (t<=10)*0.6 + (t>10 & t<=30)*0.3 + (t>30)*0.6;
yd = @(t) (t<=20)*0.6 + (t>20 & t<=40)*0.3 + (t>40)*0.6;
zd = @(t) (t<=50)*0.6 + (t>50)*0;
sayd = @(t) (t<=50)*0.5 + (t>50)*0;

% First and Second Time Derivatives of the Reference Signals
xDotd = 0; xDoubleDotd = 0; 
xDot7d = zeros(size(t)); xDoubleDot7d = 0;

yDotd = 0; yDoubleDotd = 0;
xDot9d = zeros(size(t)); xDoubleDot9d = 0;

zDotd = 0; zDoubleDotd = 0;
xDot11d = zeros(size(t)); xDoubleDot11d = 0;

%% Initialization for the Main Loop

% Define main state Vector
x = zeros(nStates,numel(t));
x(:,1) = Ics_States;

nControl = 4;   % Number of Control Inputs

U = zeros(nControl,numel(t));

nC1 = 3;                    % Number of Control Inputs of the first Subsystem
V  = zeros(nC1,numel(t));   % Each Column is a Sample

xDoubleDot1_d = 0;
xDoubleDot3_d = 0;
xDoubleDot5_d = 0;

% Treasholds
mu_phi = 0.01;
mu_theta = 0.01;
mu_say = 0.01;
mu = [mu_phi mu_theta mu_say]';
a2 = 0;a5 = 0;

for i=2:numel(t)  
    %% Control of the First Subsystem
    
    % BackStepping
    x7 = x(7,i-1);
    x9 = x(9,i-1);
    x11 = x(11,i-1);
    
    x8 = x(8,i-1);
    x10 = x(10,i-1);
    x12 = x(12,i-1);
    
    x7d = xd(t(i-1));
    x9d = yd(t(i-1));
    x11d = zd(t(i-1));
    
    ex1 = x7-x7d;
    ey1 = x9-x9d;
    ez1  = x11-x11d;
    
    % Internal Control Signals
    x8_d = -cx1*ex1 + xDot7d(i-1);
    x10_d = -cy1*ey1 + xDot9d(i-1);
    x12_d = -cz1*ez1 + xDot11d(i-1);
    
    ex2 = x8-x8_d;
    ey2 = x10-x10_d;
    ez2 = x12-x12_d;
    
    % Virtual Control Signals for Position SubSystem
    vx = -ex1-cHat_x2(i-1)*ex2-cx1*(ex2-cx1*ex1)-a9*x8+xDoubleDot7d;
    vy = -ey1-cHat_y2(i-1)*ey2-cy1*(ey2-cy1*ey1)-a10*x10+xDoubleDot9d;
    vz = -ez1-cHat_z2(i-1)*ez2-cz1*(ez2-cz1*ez1)-a11*x12+xDoubleDot11d;
    
    cHat_x2(i) = SampleTime*(gamma7*ex2^2) + cHat_x2(i-1);
    cHat_y2(i) = SampleTime*(gamma9*ey2^2) + cHat_y2(i-1);
    cHat_z2(i) = SampleTime*(gamma11*ez2^2) + cHat_z2(i-1);
    
    Ff = m*sqrt(vx^2+vy^2+(vz+g)^2);
    theta_d = atan(C(sayd(t(i-1)))*vx+S(sayd(t(i-1)))*vy/(vz+g));
    phi_d = atan(C(theta_d)*(S(sayd(t(i-1)))*vx-C(sayd(t(i-1)))*vy)/(vz+g));
    
    V(:,i) = [Ff theta_d phi_d]';   % Control Signals of the first SubSystem
    
    %% Control of the Second Subsystem
    
    x1 = x(1,i-1);
    x3 = x(3,i-1);
    x5 = x(5,i-1);
    
    x2 = x(2,i-1);
    x4 = x(4,i-1);
    x6 = x(6,i-1);
    
    x1d = phi_d;
    x3d = theta_d;
    x5d = sayd(t(i-1));
   
    x1d_dot = 0;  
    x3d_dot = 0;
    x5d_dot = 0;
    
    e_phi = x1 - x1d;
    e_theta = x3 - x3d;
    e_say = x5 - x5d;
    
    e_phi_dot = x2-x1d_dot;
    e_theta_dot = x4 - x3d_dot;
    e_say_dot = x6 - x5d_dot;
    
    %% E. Define the Sliding Surfaces and Parameter Optimization
    
    s_bar_phi = e_phi_dot + alpha_phi*e_phi+beta_phi*e_phi^(p_phi/q_phi);
    s_bar_theta = e_theta_dot + alpha_theta*e_theta+beta_theta*e_theta^(p_theta/q_theta);
    s_bar_say = e_say_dot + alpha_say*e_say+beta_say*e_say^(p_say/q_say);
    
    SBAR = [s_bar_phi s_bar_theta s_bar_say]';
    PHI = zeros(numel(SBAR),1);
    ERROR = [e_phi e_theta e_say]';
    p_q = [p_phi/q_phi p_theta/q_theta p_say/q_say]';
    
    for j=1:numel(SBAR)
        if (SBAR(j) == 0)||(SBAR(j)~=0 && abs(ERROR(j))>mu(j))
            
            PHI(j) = (abs(ERROR(j)))^(p_q(j));
        
        elseif (SBAR(j)~=0 && abs(ERROR(j))<mu(j))
            
            PHI(j) = ERROR(j);
        end
    end
    
    s_phi = e_phi_dot + alpha_phi*e_phi + beta_phi*PHI(1);
    s_theta = e_theta_dot + alpha_theta*e_theta + beta_theta*PHI(2);
    s_say = e_say_dot + alpha_say*e_say + beta_say*PHI(3);
    
    kHat_phi(i) = SampleTime*(gammaPhi*abs(s_phi)) + kHat_phi(i-1);
    kHat_theta(i) = SampleTime*(gammaTheta*abs(s_theta)) + kHat_theta(i-1);
    kHat_say(i) = SampleTime*(gammaSay*abs(s_say)) + kHat_say(i-1);
    
    %% Phi Control Signal
    
    if (s_bar_phi == 0)||(s_bar_phi~=0 && abs(e_phi)>mu_phi)

        tau_phi_eq = (1/b1)*(-(a1*x4*x6+a2*x4+a3*x2^2) + xDoubleDot1_d - ...
            alpha_phi*(s_phi-c_phi*e_phi)-...
            (s_phi-c_phi*e_phi)*(beta_phi*(p_phi/q_phi)*sign(e_phi)*(abs(e_phi))^((p_phi/q_phi)-1))-e_phi);
        
    elseif (s_bar_phi~=0 && abs(e_phi)<= mu_phi)

        tau_phi_eq = (1/b1)*(-(a1*x4*x6+a2*x4+a3*x2^2) + xDoubleDot1_d - ...
            (alpha_phi+beta_phi)*(s_phi-c_phi*e_phi)-e_phi);
    end
    
    %% Theta Control Signal
    
    if (s_bar_theta == 0)||(s_bar_theta~=0 && abs(e_theta)>mu_theta)
        
        tau_theta_eq = (1/b2)*(-(a4*x2*x6+a5*x2+a6*x4^2) + xDoubleDot3_d - ...
            alpha_theta*(s_theta-c_theta*e_theta)-...
            (s_theta-c_theta*e_theta)*(beta_theta*(p_theta/q_theta)*sign(e_theta)*...
            (abs(e_theta))^((p_theta/q_theta)-1))-e_theta);
       
    elseif(s_bar_theta~=0 && abs(e_theta)<= mu_theta)
        
        tau_theta_eq = (1/b2)*(-(a4*x2*x6+a5*x2+a6*x4^2) + xDoubleDot3_d - ...
            (alpha_theta+beta_theta)*(s_theta-c_theta*e_theta)-e_theta);
        
    end
   
    %% Say Control Signal
    
    if (s_bar_say == 0)||(s_bar_say~=0 && abs(e_say)>mu_say)
        
        tau_say_eq = (1/b3)*(-(a7*x2*x4+a8*x6^2) + xDoubleDot5_d - ...
            alpha_say*(s_say-c_say*e_say)-...
            (s_say-c_say*e_say)*(beta_say*(p_say/q_say)*sign(e_say)*...
            (abs(e_say))^((p_say/q_say)-1))-e_say);
    
    elseif (s_bar_say~=0 && abs(e_say)<= mu_say)
        
        tau_say_eq = (1/b3)*(-(a7*x2*x4+a8*x6^2) + xDoubleDot5_d - ...
            (alpha_say+beta_say)*(s_say-c_say*e_say)-e_say);
    end
    
    tau_phi_s = (1/b1)*(-kHat_phi(i)*tanh(s_phi));
    tau_theta_s = (1/b2)*(-kHat_theta(i)*tanh(s_theta));
    tau_say_s = (1/b3)*(-kHat_say(i)*tanh(s_say));
    
    tau_phi = tau_phi_eq + tau_phi_s;
    tau_theta = tau_theta_eq + tau_theta_s;
    tau_say = tau_say_eq + tau_say_s;
    
    U(:,i) = [tau_phi tau_theta tau_say Ff]';  %The Final Vector of Control Signals 

    %% A. Solve The Nonlinear Dynamic
    
    K1 = Sys_ODE_Fun(t(i-1),x(:,i-1),U(:,i));
    K2 = Sys_ODE_Fun(t(i-1)+(SampleTime/2),x(:,i-1)+SampleTime*K1/2,U(:,i));
    K3 = Sys_ODE_Fun(t(i-1)+(SampleTime/2),x(:,i-1)+SampleTime*K2/2,U(:,i));
    [K4,wr] = Sys_ODE_Fun(t(i-1)+SampleTime,x(:,i-1)+SampleTime*K3,U(:,i));
    
    a2 = -wr*Ir/Ixx;a5 = wr*Ir/Iyy;

    %% Final Response for the states
    
    x(:,i) = x(:,i-1) + (SampleTime/6)*(K1 + 2*K2 + 2*K3 + K4);
    
end

%% Plot Results
if switcher == 1
    Plot_Results(t,x,xd,yd,zd,sayd,V,U,...
        cHat_x2,cHat_y2,cHat_z2,kHat_phi,kHat_theta,kHat_say)
end
%% Save Data

X = x(7,:);
Y = x(9,:);
Z = x(11,:);

PHI = x(1,:);
THETA = x(3,:);
SAY = x(5,:);

FF = U(end,:);
TAU_PHI = U(1,:);
TAU_THETA = U(2,:);
TAU_SAY = U(3,:);

DATA.X = X;
DATA.Y = Y;
DATA.Z = Z;
DATA.PHI = PHI;
DATA.THETA = THETA;
DATA.SAY = SAY;
DATA.FF = FF;
DATA.TAU_PHI = TAU_PHI;
DATA.TAU_THETA = TAU_THETA;
DATA.TAU_SAY = TAU_SAY;

if switcher ==1
    % using AB-FTSMC
    save('DATA_TSMC','DATA')
else
    %using AB-SMC
    save('DATA_BSMC','DATA')
end

%% Comparison
if comparison ==1
    Data4 = load('DATA_BSMC');
    data4 = Data4.DATA;
    
    X_BSMC = data4.X;
    Y_BSMC = data4.Y;
    Z_BSMC = data4.Z;
    PHI_BSMC = data4.PHI;
    THETA_BSMC = data4.THETA;
    SAY_BSMC = data4.SAY;
    
    f10 = figure(10);
    subplot(3,1,1)
    plot(t,xd(t),'b','LineWidth',1)
    hold on
    grid on
    plot(t,X_BSMC,'c','LineWidth',2)
    hold on
    plot(t,x(7,:),'r','LineWidth',2)
    xlabel('T(s)','InterPreter','LateX')
    ylabel('x(m)','InterPreter','Latex')
    ylim([0 0.8])
    legend('Reference','BSMC','Proposed method')
    
    subplot(3,1,2)
    plot(t,yd(t),'b','LineWidth',2)
    hold on
    grid on
    plot(t,Y_BSMC,'c','LineWidth',2)
    hold on
    plot(t,x(9,:),'r','LineWidth',2)
    xlabel('T(s)','InterPreter','LateX')
    ylabel('y(m)','InterPreter','Latex')
    ylim([0 0.8])
    legend('Reference','BSMC','Proposed method')
    
    subplot(3,1,3)
    plot(t,zd(t),'b','LineWidth',2)
    hold on
    grid on
    plot(t,Z_BSMC,'c','LineWidth',2)
    hold on
    plot(t,x(11,:),'r','LineWidth',2)
    xlabel('T(s)','InterPreter','LateX')
    ylabel('z(m)','InterPreter','Latex')
    ylim([-0.2 0.8])
    legend('Reference','BSMC','Proposed method')
    movegui(f10,'center')
    
    f11 = figure(11);
    subplot(3,1,1)
    plot(t,V(end,:),'b','LineWidth',2)
    hold on
    grid on
    plot(t,x(1,:),'r','LineWidth',2)
    plot(t,PHI_BSMC,'c','LineWidth',2)
    legend('Reference','BSMC','Proposed Method')
    xlabel(' T(s)','InterPreter','Latex')
    ylabel('\phi (rad)')
    ylim([-0.4 0.4])
        
    subplot(3,1,2)
    plot(t,V(end-1,:),'b','LineWidth',2)
    hold on
    grid on
    plot(t,x(3,:),'r','LineWidth',2)
    plot(t,THETA_BSMC,'c','LineWidth',2)
    legend('Reference','BSMC','Proposed Method')
    xlabel(' T(s)','InterPreter','Latex')
    ylabel('\theta (rad)')
    ylim([-0.4 0.4])
        
    subplot(3,1,3)
    plot(t,feval(sayd,t),'b','LineWidth',2)
    hold on
    grid on
    plot(t,SAY_BSMC,'c','LineWidth',2)
    hold on
    plot(t,x(5,:),'r','LineWidth',2)
    legend('Reference','BSMC','Proposed Method')
    xlabel(' T(s)','InterPreter','Latex')
    ylabel('\psi (rad)')
    ylim([-0.2 0.6])
end

%% Result Plotter
function Plot_Results(t,x,xd,yd,zd,sayd,V,U,...
    cHat_x2,cHat_y2,cHat_z2,kHat_phi,kHat_theta,kHat_say)
    %% SubSystem 1
    
    nStates = 12;
    X = x(7,:);
    y = x(9,:);
    z = x(11,:);

    % Function evaluation for trajectories
    xd_Values = feval(xd,t);
    yd_Values = feval(yd,t);
    zd_Values = feval(zd,t);

    u = x(8,:);
    v = x(10,:);
    w = x(12,:);

    f1 = figure(1);
    Name = {'X_{1}','X_{2}','X_{3}','X_{4}','X_{5}','X_{6}',...
            'X_{7}','X_{8}','X_{9}','X_{10}','X_{11}','X_{12}'};

    for i=1:size(x,1)

        subplot(nStates/2,2,i)
        plot(t,x(i,:),'LineWidth',2)
        grid on
        xlabel(' Time(s)','FontWeight','Bold')
        legend(Name{i},'FontWeight','Bold')

    end

    f2 = figure(2);
    subplot(3,1,1)
    plot(t,xd_Values,'b','LineWidth',1.75)
    hold on
    grid on
    plot(t,X,'r','LineWidth',1.2)
    legend('Reference','Proposed Method')
    ylabel('x (m)','InterPreter','Latex')
    ylim([0 0.8])
    
    subplot(3,1,2)
    plot(t,yd_Values,'b','LineWidth',1.75)
    hold on
    grid on
    plot(t,y,'r','LineWidth',1.2)
    legend('Reference','Proposed Method')
    ylabel('y (m)','InterPreter','Latex')
    ylim([0 0.8])
    
    subplot(3,1,3)
    plot(t,zd_Values,'b','LineWidth',1.75)
    hold on
    grid on
    plot(t,z,'r','LineWidth',1.2)
    legend('Reference','Proposed Method')
    ylabel('z (m)','InterPreter','Latex')
    ylim([-0.2 0.8])
    
    f3 = figure(3);
    subplot(3,1,1)
    plot(t,u,'k','LinewIDTH',2)
    grid on
    xlabel('T (s)')
    ylabel('u (m/s)')
    ylim([-0.6 0.8])
    
    subplot(3,1,2)
    plot(t,v,'k','LinewIDTH',2)
    grid on
    xlabel('T (s)')
    ylabel('v (m/s)')
    ylim([-0.6 0.8])
    
    subplot(3,1,3)
    plot(t,w,'k','LinewIDTH',2)
    grid on
    xlabel('T (s)')
    ylabel('w (m/s)')
    ylim([-0.8 0.6])
    
    %% SubSystem 2

    phi = x(1,:);
    theta = x(3,:);
    say = x(5,:);

    p = x(2,:);
    q = x(4,:);
    r = x(6,:);

    f4 = figure(4);
    subplot(3,1,1)
    plot(t,V(end,:),'b','LineWidth',1.75)
    hold on
    grid on
    plot(t,phi,'r','LineWidth',1.2)
    legend('Reference','Proposed Method')
    xlabel(' T(s)','InterPreter','Latex')
    ylabel('\phi (rad)')
    ylim([-0.4 0.4])
    
    subplot(3,1,2)
    plot(t,V(end-1,:),'b','LineWidth',1.75)
    hold on
    grid on
    plot(t,theta,'r','LineWidth',1.2)
    legend('Reference','Proposed Method')
    xlabel(' T(s)','InterPreter','Latex')
    ylabel('\theta (rad)')
    ylim([-0.4 0.4])
    
    subplot(3,1,3)
    plot(t,feval(sayd,t),'b','LineWidth',1.75)
    hold on
    grid on
    plot(t,say,'r','LineWidth',1.2)
    legend('Reference','Proposed Method')
    xlabel(' T(s)','InterPreter','Latex')
    ylabel('\psi (rad)')
    ylim([-0.2 0.6])
    
    f5 = figure(5);
    subplot(3,1,1)
    plot(t,p,'k','LineWidth',2)
    grid on
    xlabel('T(s)','InterPreter','LAtEx')
    ylabel('p (rad/s)')
    ylim([-2 1.5])
    
    subplot(3,1,2)
    plot(t,q,'k','LineWidth',2)
    grid on
    xlabel('T(s)','InterPreter','LAtEx')
    ylabel('q (rad/s)','InterPreter','Latex')
    ylim([-1.5 2])
    
    subplot(3,1,3)
    plot(t,r,'k','LineWidth',2)
    grid on
    xlabel('T(s)','InterPreter','LAtEx')
    ylabel('r (rad/s)','InterPreter','Latex')
    ylim([-4 3])
    
    f6 = figure(6);
    subplot(4,1,1)
    plot(t,V(1,:),'K','LineWidth',2)
    xlabel('T(s)','InterPreter','Latex')
    grid on
    ylabel('F_{f} (N)','FontWeight','bOLd')
    ylim([2 10])
    
    subplot(4,1,2)
    plot(t,U(1,:),'K','LineWidth',2)
    xlabel('T(s)','InterPreter','Latex')
    grid on
    ylabel('\tau_{\phi} (N.m)')
    ylim([-0.15 0.1])
    
    subplot(4,1,3)
    plot(t,U(2,:),'K','LineWidth',2)
    xlabel('T(s)','InterPreter','Latex')
    grid on
    ylabel('\tau_{\theta} (N.m)')
    ylim([-0.1 0.15])
    
    subplot(4,1,4)
    plot(t,U(3,:),'K','LineWidth',2)
    xlabel('T(s)','InterPreter','Latex')
    grid on
    ylabel('\tau_{\psi} (N.m)')
    ylim([-0.4 0.4])
    
    f7 = figure(7);
    Data4 = load('DATA_BSMC');
    data4 = Data4.DATA;
    X_BSMC = data4.X;
    Y_BSMC = data4.Y;
    Z_BSMC = data4.Z;
    
    plot3(xd_Values,yd_Values,zd_Values,'k--','LineWidth',2)
    grid on
    hold on
    plot3(X_BSMC,Y_BSMC,Z_BSMC,'b','LineWidth',1.8)
    hold on
    plot3(X,y,z,'r','LInEwIDTh',1.6)
    xlabel('x (m)','InterPreter','Latex')
    ylabel('y (m)','InterPreter','Latex')
    zlabel('z (m)','InterPreter','Latex')
    legend('Reference','B-SMC','Proposed','InterPreter','latex')

    f8 = figure(8);
    subplot(3,1,1)
    plot(t,cHat_x2,'k','LineWidth',2)
    grid on
    xlabel('T(s)','FontWeight','Bold')
    ylabel('C^\wedge_{x_{2}}','FontWeight','Bold')

    subplot(3,1,2)
    plot(t,cHat_y2,'k','LineWidth',2)
    grid on
    xlabel('T(s)','FontWeight','Bold')
    ylabel('C^\wedge_{y_{2}}','FontWeight','Bold')

    subplot(3,1,3)
    plot(t,cHat_z2,'k','LineWidth',2)
    grid on
    xlabel('T(s)','FontWeight','Bold')
    ylabel('C^\wedge_{z_{2}}','FontWeight','Bold')

    f9 = figure(9);
    subplot(3,1,1)
    plot(t,kHat_phi,'k','LineWidth',2)
    grid on
    xlabel('T(s)','Interpreter','latex')
    ylabel('K^\wedge_{\phi}')

    subplot(3,1,2)
    plot(t,kHat_theta,'k','LineWidth',2)
    grid on
    xlabel('T(s)','Interpreter','latex')
    ylabel('K^\wedge_{\theta}')

    subplot(3,1,3)
    plot(t,kHat_say,'k','LineWidth',2)
    grid on
    xlabel('T(s)','Interpreter','latex')
    ylabel('K^\wedge_{\psi}')
    
    %% Move Figures
    
    nFigures = 9;   % Number of Figures
    Location = {'north','west','south','east',...
                 'northeast','northwest','southeast','southwest',...
                 'center'};
    
    Cell_Fig = {f1,f2,f3,f4,f5,f6,f7,f8,f9};
    for M=1:nFigures
        
        movegui(Cell_Fig{M},Location{M})
    end
    
end

%% Plants ODEs
function [xDot,wr] = Sys_ODE_Fun(t,x,u)
    %% State Vector and control inputs

    % Control Inputs
    tau_phi = u(1);
    tau_theta = u(2);
    tau_say = u(3);
    Ff = u(4);
    
    %% System Parameters
    
    Parameters = Params_Func();
    
    g = Parameters.g;
    m = Parameters.m;
    Ixx = Parameters.Ixx;
    Iyy = Parameters.Iyy;
    Izz = Parameters.Izz;
    Ir = Parameters.Ir;
    k1 = Parameters.k1;
    k2 = Parameters.k2;
    k3 = Parameters.k3;
    k4 = Parameters.k4;
    k5 = Parameters.k5;
    k6 = Parameters.k6;
    kax = k1;kay = k2;kaz = k3;
    kdx = k4;kdy = k5;kdz = k6;
    kp = Parameters.kp;
    cd = Parameters.cd;
    ld = 0.4;   % Quad Length
    bd = kp;
    
    C = @(x) cos(x);
    S = @(x) sin(x);
    
    Transformation_Matrix = [cd -cd cd -cd
                             -ld*bd 0 ld*bd 0
                             0 -ld*bd 0 ld*bd
                             bd bd bd bd];
        
    Sol_Vector = [tau_say tau_theta tau_phi Ff]';
    Sol = linsolve(Transformation_Matrix,Sol_Vector);

    w1s = Sol(1);
    w2s = Sol(2);
    w3s = Sol(3);
    w4s = Sol(4);
    
    W = sqrt([w1s w2s w3s w4s]);
    w1 = W(1);
    w2 = W(2);
    w3 = W(3);
    w4 = W(4);
   
    %% System Dynamics
    
    wr = -w1 - w2 + w3 + w4;
    a1 = (Iyy-Izz)/Ixx;
    a2 = -wr*Ir/Ixx;
    a3 = -kax/Ixx;
    a4 = (Izz-Ixx)/Iyy;
    a5 = wr*Ir/Iyy;
    a6 = -kay/Iyy;
    a7 = (Ixx-Iyy)/Izz;
    a8 = -kaz/Izz;
    a9 = -kdx/m;
    a10 = -kdy/m;
    a11 = -kdz/m;
    b1 = 1/Ixx;b2 = 1/Iyy;b3 = 1/Izz;
    
    xDot = [x(2)
            a1*x(4)*x(6) + a2*x(4) + a3*x(2)^2 + b1*tau_phi
            x(4)
            a4*x(2)*x(6) + a5*x(2) + a6*x(4)^2 + b2*tau_theta
            x(6)
            a7*x(2)*x(4) + a8*x(6)^2 + b3*tau_say
            x(8)
            a9*x(8) + (Ff/m)*(C(x(1))*S(x(3))*C(x(5)) + S(x(1))*S(x(5)))
            x(10)
            a10*x(10) + (Ff/m)*(C(x(1))*S(x(3))*S(x(5)) - S(x(1))*C(x(5)))
            x(12)
            a11*x(12) - g + (Ff/m)*C(x(1))*C(x(3))];
end

%% System Parameters
function Parameters = Params_Func()

    Parameters.g = 9.81;
    Parameters.m = 0.744;
    Parameters.Ixx = 3.827e-3;
    Parameters.Iyy = 3.827e-3;
    Parameters.Izz = 7.6566e-3;
    Parameters.Ir = 2.8385e-5;
    
    Parameters.k1 = 5.567e-4;
    Parameters.k2 = 5.567e-4;
    Parameters.k3 = 5.567e-4;
    Parameters.k4 = 5.567e-4;
    Parameters.k5 = 5.567e-4;
    Parameters.k6 = 5.567e-4;
    Parameters.kp = 2.9842e-3;
    Parameters.cd = 3.323e-2;

end