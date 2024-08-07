function [dXdt,u,GRFF,GRFB] = dyn_back_stance_front_jump(t,X,p)
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
% Feedforward force
s = t / 0.3;        % stance phase parametrization s = [0, 1]
% Force profile using Bezier polynomials
FBy = polyval_bz([130 130 130 130 130], s);
FBx = polyval_bz([30 30 20 20 20], s);

uB = -JBtoe'*[FBx; FBy];

qfh_TD = -1/3*pi; % Desired touchdown angle
qfk_TD = 0.5*pi;

ufh = 50*(qfh_TD-q(4)) + 2*(0-qdot(4));
ufk = 50*(qfk_TD-q(5)) + 2*(0-qdot(5));

if ufh > 15
    ufh = 15;
end
if ufh < -15
    ufh = -15;
end
if ufk > 15
    ufk = 15;
end
if ufk < -15
    ufk = -15;
end

u = [ufh;ufk;uB(6);uB(7)];

% Solve the linear system:
% De * ddq + Ce * dq + Ge = JF' * FGRF + JB' * BGRF + Be * u (7 eqns)
% Jhc * ddq + dJhc * dq = 0 (2 eqns)
% [De  -JBtoe'] * [ddq ] = [Be*u - Ce*dq - Ge]
% [JBtoe  0   ]   [GRFB]   [-JdotBtoe*dq     ]
% unknowns: ddq(7x1), GRF(4x1) 
% control: u(4x1)
Amat = [De -JBtoe'; 
        JBtoe zeros(2,2)];
bvec = [Be*u - Ce*qdot - Ge; 
        -JdotBtoe*qdot];

ddqu = Amat \ bvec;
ddq = ddqu(1:7);
GRFF = [0;0];
GRFB = ddqu(8:9);

FFy = GRFF(2);
FBy = GRFB(2);

dXdt = [qdot; ddq];

end

