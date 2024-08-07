function [zeroCrossing,isterminal,direction] = touch_down(t,X,p)

params = p.params;

q = X(1:7);
pBtoe = fcn_pBtoe(q,params);
pFtoe = fcn_pFtoe(q,params);
if pBtoe(2) <= pFtoe(2)
    zeroCrossing =  pBtoe(2);
else
    zeroCrossing =  pFtoe(2);
end
isterminal   =  1;
direction    =  -1;