function [dXdt, pFtoe, pBtoe,FFspring,FBspring] = dyn_aerial(t,X,p)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

global pFtoe0 pBtoe0
global KF r0F thetaF_TD KB r0B thetaB_TD
params = p.params;
qdot = X(4:6);

% Calculate acceleration and angular acceleration
xdotdot = 0;
ydotdot = - params(1);
thetadotdot = 0;

qdotdot = [xdotdot; ydotdot; thetadotdot];

dXdt = [qdot; qdotdot];

pFH = fcn_pFH(X,params);
thetaFH = X(3);
T0FH = [cos(thetaFH) -sin(thetaFH) pFH(1); sin(thetaFH) cos(thetaFH) pFH(2); 0 0 1];
RFHFtoe = [cos(thetaF_TD) -sin(thetaF_TD); sin(thetaF_TD) cos(thetaF_TD)];
TFHFtoe = [RFHFtoe RFHFtoe*[0; -r0F]; 0 0 1];
T0Ftoe = T0FH * TFHFtoe;
pFtoe = T0Ftoe(1:2,3);

pBH = fcn_pBH(X,params);
thetaBH = X(3);
T0BH = [cos(thetaBH) -sin(thetaBH) pBH(1); sin(thetaBH) cos(thetaBH) pBH(2); 0 0 1];
RBHBtoe = [cos(thetaB_TD) -sin(thetaB_TD); sin(thetaB_TD) cos(thetaB_TD)];
TBHBtoe = [RBHBtoe RBHBtoe*[0; -r0B]; 0 0 1];
T0Btoe = T0BH * TBHBtoe;
pBtoe = T0Btoe(1:2,3);


FFspring = 0;
FBspring = 0;
end

