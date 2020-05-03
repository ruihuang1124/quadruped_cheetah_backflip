function dX = SLIP_Flight(t,X)

global g m r0 K theta_TD
%Current state
x = X(1);   
y = X(2);  
x_dot = X(3);   
y_dot = X(4); 

%Balistic Dynamics
x_dotdot = ;   
y_dotdot = ; 

dX = [x_dot; y_dot; x_dotdot; y_dotdot];
end