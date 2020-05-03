function dX = SLIP_Stance(t,X)

global g m r0 K theta_TD
%Current state
r = X(1);   
theta = X(2);  
r_dot = X(3);   
theta_dot = X(4); 

%Stance dynamics:
r_dotdot = ;
theta_dotdot = ;

dX = [r_dot; theta_dot; r_dotdot; theta_dotdot];
end