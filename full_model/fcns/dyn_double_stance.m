function [dXdt] = dyn_double_stance(t,X,p)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
t

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
s = t / 0.2;        % stance phase parametrization s = [0, 1]
% Force profile using Bezier polynomials
FFy = polyval_bz([0 120 120 120 0], s);
FFx = polyval_bz([0 0 0 0 0], s);

FBy = polyval_bz([0 40 40 40 0], s);
FBx = polyval_bz([0 0 0 0 0], s);

uF = -JFtoe'*[FFx; FFy];
uB = -JBtoe'*[FBx; FBy];
u = [uF(4);uF(5);uB(6);uB(7)];

% Solve the linear system:
% De * ddq + Ce * dq + Ge = JF' * FGRF + JB' * BGRF + Be * u (7 eqns)
% Jhc * ddq + dJhc * dq = 0 (4 eqns)
% [De  -JFtoe' -JBtoe'] * [ddq ] = [Be*u - Ce*dq - Ge]
% [JFtoe  0       0   ]   [GRFF]   [-JdotFtoe*dq     ]
% [JBtoe  0       0   ]   [GRFB]   [-JdotBtoe*dq     ]
% unknowns: ddq(7x1), GRF(4x1) 
% control: u(4x1)
Amat = [De -JFtoe' -JBtoe'; 
        JFtoe zeros(2,4);
        JBtoe zeros(2,4)];
bvec = [Be*u - Ce*qdot - Ge; 
        -JdotFtoe*qdot;
        -JdotBtoe*qdot];

ddqu = Amat \ bvec;
ddq = ddqu(1:7);
GRFF = ddqu(8:9);
GRFB = ddqu(10:11);

dXdt = [qdot; ddq];

end

