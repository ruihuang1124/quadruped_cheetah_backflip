clc
clear all
close all

addpath gen
addpath fcns
addpath animate

p = get_params;

% Initial condition for floating base center of mass x, y, theta,
% qfronthip, qfrontknee, qbackhip, and qbackknee
q0 = [0; 0.209+0.18-0.1; 0.15];
q0dot = [0; 0; 0; 0; 0; 0; 0];

% Solve for front toe inverse kinematics
pFtoe0 = [0.25; 0];
qf = fcn_IKfront(q0, p.params, pFtoe0);
q0 = [q0; qf];

% Solve for back toe inverse kinematics
pBtoe0 = [-0.2; 0];
qb = fcn_IKback(q0, p.params, pBtoe0);
q0 = [q0; qb];
ic = [q0; q0dot];

% Reduced spring model parameters for front leg and back leg
KF = 8000;  
r0F = 0.209+0.18;

KB = 8000;
r0B = 0.209+0.18;

%Ploting the robot in the initial configuration:
figure(1)
plot_robot(ic,p);

%%

%%

global g m r0 K theta_TD
%Parameters:
g = 9.81;   %gravity
m = 5;      %SLIP mass
r0 = 1;     %SLIP resting length
K = 8000;   %SLIP stiffness
N_hops = 5; %Number of hops to simulate

%Initial conditions for simulation
x0 = 0;
y0 = 1.5*r0;
x_dot0 = 0;
y_dot0 = 0;
X0 = [x0; y0; x_dot0; y_dot0];

for i = 1:N_hops
    
    %Controller to choose step location:
    Kf = 0;      %Foot placement gain
    v_des = 0;	%Desired horizontal velocity
    %Desired touch-down angle proportional to speed error:
    theta_TD = Kf*(x_dot0 - v_des); 
    
    %Flight dynamics: [x; y; x_dot; y_dot]
    options = odeset('Events',@touch_down); 
    [Tout_flight,Xout_flight] = ode45(@SLIP_Flight,[0 10],X0,options);
    
    %-----------------------------------------------------------------
    %Stance dynamics: [r; theta; r_dot; theta_dot]    
    X0 = 0;  %Initial conditions for stance dynamics
    options = odeset('Events',@take_off); 
    [Tout_stance,Xout_stance] = ode45(@SLIP_Stance,[0 10],X0,options);

    %-----------------------------------------------------------------
    %Flight initial conditions:
    X0 = 0;  %Initial conditions for flight dynamics

end



