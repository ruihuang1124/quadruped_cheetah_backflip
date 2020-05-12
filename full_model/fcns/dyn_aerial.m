function [dXdt,u,GRFF,GRFB] = dyn_aerial(t,X,p)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
t

global FFy FBy

params = p.params;
q = X(1:7);
qdot = X(8:14);

De = fcn_De(q,params);
Ce = fcn_Ce(q,qdot,params);
Ge = fcn_Ge(q,params);
Be = fcn_Be(q,params);

% Holonomic constraints
JBtoe = fcn_JBtoe(q,params);
JdotBtoe = fcn_JdotBtoe(q,qdot,params);
JFtoe = fcn_JFtoe(q,params);
JdotFtoe = fcn_JdotFtoe(q,qdot,params);


%% Controller

qfh_TD = -1/3*pi; % Desired touchdown angle
qfk_TD = 0.5*pi;

ufh = 50*(qfh_TD-q(4)) + 2*(0-qdot(4));
ufk = 50*(qfk_TD-q(5)) + 2*(0-qdot(5));

qbh_TD = -1/3*pi; % Desired touchdown angle
qbk_TD = 0.5*pi;

ubh = 50*(qbh_TD-q(6)) + 2*(0-qdot(6));
ubk = 50*(qbk_TD-q(7)) + 2*(0-qdot(7));

u = [ufh;ufk;ubh;ubk];

% Solve the linear system:
% De * ddq + Ce * dq + Ge = Be * u (7 eqns)
% [De ] * [ddq ] = [Be*u - Ce*dq - Ge]
% unknowns: ddq(7x1)
% control: u(4x1)
Amat = De;
bvec = Be*u - Ce*qdot - Ge;

ddqu = Amat \ bvec;
ddq = ddqu(1:7);
GRFF = [0;0];
GRFB = [0;0];

FFy = GRFF(2);
FBy = GRFB(2);

dXdt = [qdot; ddq];

end

