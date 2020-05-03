function [value, isterminal, direction] = touch_down(t, X)

global g m r0 K theta_TD
%Current state
x = X(1);   
y = X(2);  
x_dot = X(3);   
y_dot = X(4); 

%Write here the guard funtion to touch-down: 
%Event occurs when value <= 0
value      = ;
isterminal = 1;   % Stop the integration
direction  = -1;
end



