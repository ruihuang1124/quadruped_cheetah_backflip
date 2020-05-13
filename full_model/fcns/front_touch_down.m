function [zeroCrossing,isterminal,direction] = front_touch_down(t,X,p)

params = p.params;

q = X(1:7);
pFtoe = fcn_pFtoe(q,params);

zeroCrossing =  pFtoe(2);
isterminal   =  1;
direction    =  -1;