function [value, isterminal, direction] = take_off(t, X)

global g m r0 K theta_TD
%Current state
r = X(1);   
theta = X(2);  
r_dot = X(3);   
theta_dot = X(4); 

%Write here the guard funtion to take-off: 
%Event occurs when value <= 0
value      = ;  
isterminal = 1;   % Stop the integration
direction = -1;
end



