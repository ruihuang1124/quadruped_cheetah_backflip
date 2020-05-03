function [dXdt,pFtoe,pBtoe,FFspring,FBspring] = dyn_back_stance(t,X,p)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

global pFtoe0 pBtoe0
global KF r0F thetaF_TD KB r0B
params = p.params;
qdot = X(4:6);

MB = params(5);
IB = params(8);

% Calculate and decompose back spring force
pBH = fcn_pBH(X,params);
r = sqrt((pBH(1)-pBtoe0(1))^2 + (pBH(2)-pBtoe0(2))^2);
FBspring = (r0B-r)*KB;
FBspringx = (pBH(1)-pBtoe0(1))/r*FBspring;
FBspringy = (pBH(2)-pBtoe0(2))/r*FBspring;
FFspring = 0;

% Calculate back spring torque
pB = fcn_pB(X,params);
tauB = cross([pBH-pB; 0], [FBspringx, FBspringy, 0]);
tauB = tauB(3);

% Calculate acceleration and angular acceleration
xdotdot = FBspringx / MB;
ydotdot = FBspringy / MB - params(1);
thetadotdot = tauB / IB;

qdotdot = [xdotdot; ydotdot; thetadotdot];

dXdt = [qdot; qdotdot];
pBtoe = pBtoe0;

pFH = fcn_pFH(X,params);
thetaFH = X(3);
T0FH = [cos(thetaFH) -sin(thetaFH) pFH(1); sin(thetaFH) cos(thetaFH) pFH(2); 0 0 1];
RFHFtoe = [cos(thetaF_TD) -sin(thetaF_TD); sin(thetaF_TD) cos(thetaF_TD)];
TFHFtoe = [RFHFtoe RFHFtoe*[0; -r0F]; 0 0 1];
T0Ftoe = T0FH * TFHFtoe;
pFtoe = T0Ftoe(1:2,3);
end

