function [value, isterminal, direction] = front_take_off(t,X,p)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

global r0F pFtoe0
params = p.params;

%Current state
pFH = fcn_pFH(X,params);
r = sqrt((pFH(1)-pFtoe0(1))^2 + (pFH(2)-pFtoe0(2))^2);

%Write here the guard funtion to take-off: 
%Event occurs when value <= 0
value      = r0F - r;  
isterminal = 1;   % Stop the integration
direction = -1;
end

