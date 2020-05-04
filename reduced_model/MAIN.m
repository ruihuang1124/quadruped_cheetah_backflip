clc
clear all
close all

addpath gen
addpath fcns
addpath animate

p = get_params;

% Initial condition for floating base center of mass x, y, theta,
% qfronthip, qfrontknee, qbackhip, and qbackknee
q0 = [0; 0.209+0.18-0.11; -0.25];
q0dot = [0; 0; 0];
ic = [q0; q0dot];

% Initial condition for toe positions
global pFtoe0 pBtoe0
pFtoe0 = [0.25; 0];
pBtoe0 = [-0.19; 0];

% Reduced spring model parameters for front leg and back leg
global KF r0F thetaF_TD KB r0B thetaB_TD
KF = 6400;  
r0F = 0.209+0.18-0.1;
thetaF_TD = 0.7;

KB = 700;
r0B = 0.209+0.18-0.1;
thetaB_TD = 0;

%Ploting the robot in the initial configuration:
% plot_robot(ic,pFtoe0,pBtoe0,p);

%%
tstart = 0;
tfinal = 10;
tout = tstart;
Xout = ic';
pFtoeout = pFtoe0';
pBtoeout = pBtoe0';
FFspringout = 0;
FBspringout = 0;

% Double stance phase
options = odeset('Events',@(t,X)front_take_off(t,X,p),'MaxStep',1e-3);
[t,X] = ode45(@(t,X)dyn_double_stance(t,X,p),[tstart tfinal],ic,options);

% log
nt = length(t);
tout = [tout; t(2:nt)];
Xout = [Xout; X(2:nt,:)];
for tstep = 2:nt
    [~,pFtoe,pBtoe,FFspring,FBspring] = dyn_double_stance(t(tstep),X(tstep,:)',p);
    pFtoeout = [pFtoeout; pFtoe'];
    pBtoeout = [pBtoeout; pBtoe'];
    FFspringout = [FFspringout; FFspring];
    FBspringout = [FBspringout; FBspring];
end


% Back stance phase
options = odeset('Events',@(t,X)back_take_off(t,X,p),'MaxStep',1e-3);
[t,X] = ode45(@(t,X)dyn_back_stance(t,X,p),[tout(end) tfinal],Xout(end,:),options);

% log
nt = length(t);
tout = [tout; t(2:nt)];
Xout = [Xout; X(2:nt,:)];
for tstep = 2:nt
    [~,pFtoe,pBtoe,FFspring,FBspring] = dyn_back_stance(t(tstep),X(tstep,:)',p);
    pFtoeout = [pFtoeout; pFtoe'];
    pBtoeout = [pBtoeout; pBtoe'];
    FFspringout = [FFspringout; FFspring];
    FBspringout = [FBspringout; FBspring];
end

% Aerial phase
options = odeset('Events',@(t,X)front_touch_down(t,X,p),'MaxStep',1e-3);
[t,X] = ode45(@(t,X)dyn_aerial(t,X,p),[tout(end) tfinal],Xout(end,:),options);

% log
nt = length(t);
tout = [tout; t(2:nt)];
Xout = [Xout; X(2:nt,:)];
for tstep = 2:nt
    [~,pFtoe,pBtoe,FFspring,FBspring] = dyn_aerial(t(tstep),X(tstep,:)',p);
    pFtoeout = [pFtoeout; pFtoe'];
    pBtoeout = [pBtoeout; pBtoe'];
    FFspringout = [FFspringout; FFspring];
    FBspringout = [FBspringout; FBspring];
end

global pFtoeTD
pFtoeTD = pFtoeout(end,:);

% Front stance phase
options = odeset('Events',@(t,X)back_touch_down(t,X,p),'MaxStep',1e-3);
[t,X] = ode45(@(t,X)dyn_front_stance(t,X,p),[tout(end) tfinal],Xout(end,:),options);

% log
nt = length(t);
tout = [tout; t(2:nt)];
Xout = [Xout; X(2:nt,:)];
for tstep = 2:nt
    [~,pFtoe,pBtoe,FFspring,FBspring] = dyn_front_stance(t(tstep),X(tstep,:)',p);
    pFtoeout = [pFtoeout; pFtoe'];
    pBtoeout = [pBtoeout; pBtoe'];
    FFspringout = [FFspringout; FFspring];
    FBspringout = [FBspringout; FBspring];
end

animate_robot(tout,Xout,pFtoeout,pBtoeout,FFspringout,FBspringout,p);
