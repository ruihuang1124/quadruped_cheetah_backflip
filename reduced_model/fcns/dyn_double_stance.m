function [dXdt,pFtoe,pBtoe,FFspring,FBspring] = dyn_double_stance(t,X,p)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

global pFtoe0 pBtoe0
global KF r0F KB r0B
params = p.params;
qdot = X(4:6);

MB = params(5);
IB = params(8);

% Calculate and decompose front spring force
pFH = fcn_pFH(X,params);
r = sqrt((pFH(1)-pFtoe0(1))^2 + (pFH(2)-pFtoe0(2))^2);
FFspring = (r0F-r)*KF;
if FFspring < 0
    FFspring = 0;
end
FFspringx = (pFH(1)-pFtoe0(1))/r*FFspring;
FFspringy = (pFH(2)-pFtoe0(2))/r*FFspring;

% Calculate and decompose back spring force
pBH = fcn_pBH(X,params);
r = sqrt((pBH(1)-pBtoe0(1))^2 + (pBH(2)-pBtoe0(2))^2);
FBspring = (r0B-r)*KB;
if FBspring < 0
    FBspring = 0;
end
FBspringx = (pBH(1)-pBtoe0(1))/r*FBspring;
FBspringy = (pBH(2)-pBtoe0(2))/r*FBspring;

% Calculate front spring torque
pB = fcn_pB(X,params);
tauF = cross([pFH-pB; 0], [FFspringx, FFspringy, 0]);
tauF = tauF(3);

% Calculate back spring torque
tauB = cross([pBH-pB; 0], [FBspringx, FBspringy, 0]);
tauB = tauB(3);

% Calculate acceleration and angular acceleration
xdotdot = (FFspringx + FBspringx) / MB;
ydotdot = (FFspringy + FBspringy) / MB - params(1);
thetadotdot = (tauF + tauB) / IB;

qdotdot = [xdotdot; ydotdot; thetadotdot];

dXdt = [qdot; qdotdot];
pFtoe = pFtoe0;
pBtoe = pBtoe0;

end

