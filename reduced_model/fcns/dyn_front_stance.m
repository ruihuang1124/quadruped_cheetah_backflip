function [dXdt,pFtoe,pBtoe,FFspring,FBspring] = dyn_front_stance(t,X,p)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

global pFtoeTD pBtoe0
global KF r0F KB r0B thetaB_TD
params = p.params;
qdot = X(4:6);

MB = params(5);
IB = params(8);

% Calculate and decompose front spring force
pFH = fcn_pFH(X,params);
r = sqrt((pFH(1)-pFtoeTD(1))^2 + (pFH(2)-pFtoeTD(2))^2);
FFspring = (r0F-r)*KF;
FFspringx = (pFH(1)-pFtoeTD(1))/r*FFspring;
FFspringy = (pFH(2)-pFtoeTD(2))/r*FFspring;
FBspring = 0;

pBH = fcn_pBH(X,params);

% Calculate front spring torque
pB = fcn_pB(X,params);
tauF = cross([pFH-pB; 0], [FFspringx, FFspringy, 0]);
tauF = tauF(3);

% Calculate acceleration and angular acceleration
xdotdot = FFspringx / MB;
ydotdot = FFspringy / MB - params(1);
thetadotdot = tauF / IB;

qdotdot = [xdotdot; ydotdot; thetadotdot];

dXdt = [qdot; qdotdot];
pFtoe = pFtoeTD.';

thetaBH = X(3);
T0BH = [cos(thetaBH) -sin(thetaBH) pBH(1); sin(thetaBH) cos(thetaBH) pBH(2); 0 0 1];
RBHBtoe = [cos(thetaB_TD) -sin(thetaB_TD); sin(thetaB_TD) cos(thetaB_TD)];
TBHBtoe = [RBHBtoe RBHBtoe*[0; -r0B]; 0 0 1];
T0Btoe = T0BH * TBHBtoe;
pBtoe = T0Btoe(1:2,3);

end


