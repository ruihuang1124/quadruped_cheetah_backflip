clc
clear all
close all

addpath gen
addpath fcns
addpath animate

p = get_params;

% Initial condition for floating base center of mass x, y, theta,
% qfronthip, qfrontknee, qbackhip, and qbackknee
q0 = [0; 0.209+0.18-0.26; -0.1];
q0dot = [0; 0; 0; 0; 0; 0; 0];

% Solve for front toe inverse kinematics
global pFtoe0
pFtoe0 = [0.18; 0];
qf = fcn_IKfront(q0, p.params, pFtoe0);
q0 = [q0; qf];

% Solve for back toe inverse kinematics
global pBtoe0
pBtoe0 = [-0.18; 0];
qb = fcn_IKback(q0, p.params, pBtoe0);
q0 = [q0; qb];
ic = [q0; q0dot];

%Ploting the robot in the initial configuration:
plot_robot(ic,p);

%%
tstart = 0;
tfinal = 1;
tout = tstart;
Xout = ic';
uout = [0;0;0;0]';
GRFFout = [0;0]';
GRFBout = [0;0]';

global FFy FBy

% Double stance phase
options = odeset('Events',@(t,X)front_take_off(t,X,p),'MaxStep',1e-3);
[t,X] = ode45(@(t,X)dyn_double_stance(t,X,p),[tstart 0.5],ic, options);

% log
nt = length(t);
tout = [tout; t(2:nt)];
Xout = [Xout; X(2:nt,:)];
for tstep = 2:nt
    [~,u,GRFF,GRFB] = dyn_double_stance(t(tstep),X(tstep,:)',p);
    uout = [uout; u'];
    GRFFout = [GRFFout; GRFF'];
    GRFBout = [GRFBout; GRFB'];
end

%% Back stance phase
options = odeset('Events',@(t,X)back_take_off(t,X,p),'MaxStep',1e-3);
[t,X] = ode45(@(t,X)dyn_back_stance(t,X,p),[tout(end) tfinal],Xout(end,:),options);

% log
nt = length(t);
tout = [tout; t(2:nt)];
Xout = [Xout; X(2:nt,:)];
for tstep = 2:nt
    [~,u,GRFF,GRFB] = dyn_back_stance(t(tstep),X(tstep,:)',p);
    uout = [uout; u'];
    GRFFout = [GRFFout; GRFF'];
    GRFBout = [GRFBout; GRFB'];
end

%% Aerial phase
options = odeset('Events',@(t,X)front_touch_down(t,X,p),'MaxStep',1e-3);
[t,X] = ode45(@(t,X)dyn_aerial(t,X,p),[tout(end) tfinal],Xout(end,:),options);

% log
nt = length(t);
tout = [tout; t(2:nt)];
Xout = [Xout; X(2:nt,:)];
for tstep = 2:nt
    [~,u,GRFF,GRFB] = dyn_aerial(t(tstep),X(tstep,:)',p);
    uout = [uout; u'];
    GRFFout = [GRFFout; GRFF'];
    GRFBout = [GRFBout; GRFB'];
end

animate_robot(tout,Xout,uout,GRFFout,GRFBout,p)

options = odeset('Events',@(t,X)back_touch_down(t,X,p),'MaxStep',1e-3);
[t,X] = ode45(@(t,X)dyn_front_stance(t,X,p),[tout(end) tfinal],Xout(end,:),options);

% log
nt = length(t);
tout = [tout; t(2:nt)];
Xout = [Xout; X(2:nt,:)];
for tstep = 2:nt
    [~,u,GRFF,GRFB] = dyn_front_stance(t(tstep),X(tstep,:)',p);
    uout = [uout; u'];
    GRFFout = [GRFFout; GRFF'];
    GRFBout = [GRFBout; GRFB'];
end
