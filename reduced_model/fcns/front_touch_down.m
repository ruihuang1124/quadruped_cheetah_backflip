function [value, isterminal, direction] = front_touch_down(t,X,p)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here

global r0F thetaF_TD

params = p.params;
pFH = fcn_pFH(X,params);
thetaFH = X(3);
T0FH = [cos(thetaFH) -sin(thetaFH) pFH(1); sin(thetaFH) cos(thetaFH) pFH(2); 0 0 1];
RFHFtoe = [cos(thetaF_TD) -sin(thetaF_TD); sin(thetaF_TD) cos(thetaF_TD)];
TFHFtoe = [RFHFtoe RFHFtoe*[0; -r0F]; 0 0 1];
T0Ftoe = T0FH * TFHFtoe;
pFtoe = T0Ftoe(1:2,3);

%Write here the guard funtion to touch-down: 
%Event occurs when value <= 0
value      = pFtoe(2);
isterminal = 1;   % Stop the integration
direction  = -1;
end

