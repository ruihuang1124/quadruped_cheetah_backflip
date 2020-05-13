function [zeroCrossing,isterminal,direction] = back_touch_down(t,X,p)

params = p.params;

q = X(1:7);
pBtoe = fcn_pBtoe(q,params);

zeroCrossing =  pBtoe(2);
isterminal   =  1;
direction    =  -1;