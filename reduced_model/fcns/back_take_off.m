function [value, isterminal, direction] = back_take_off(t,X,p)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

global r0B pBtoe0
params = p.params;

%Current state
pBH = fcn_pBH(X,params);
r = sqrt((pBH(1)-pBtoe0(1))^2 + (pBH(2)-pBtoe0(2))^2);

%Write here the guard funtion to take-off: 
%Event occurs when value <= 0
value      = r0B - r;  
isterminal = 1;   % Stop the integration
direction = -1;
end
