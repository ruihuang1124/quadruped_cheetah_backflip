function [value, isterminal, direction] = back_touch_down(t,X,p)
%UNTITLED12 Summary of this function goes here
%   Detailed explanation goes here

global r0B thetaB_TD

params = p.params;
pBH = fcn_pBH(X,params);
thetaBH = X(3);
T0BH = [cos(thetaBH) -sin(thetaBH) pBH(1); sin(thetaBH) cos(thetaBH) pBH(2); 0 0 1];
RBHBtoe = [cos(thetaB_TD) -sin(thetaB_TD); sin(thetaB_TD) cos(thetaB_TD)];
TBHBtoe = [RBHBtoe RBHBtoe*[0; -r0B]; 0 0 1];
T0Btoe = T0BH * TBHBtoe;
pBtoe = T0Btoe(1:2,3);

%Write here the guard funtion to touch-down: 
%Event occurs when value <= 0
value      = pBtoe(2);
isterminal = 1;   % Stop the integration
direction  = -1;
end

