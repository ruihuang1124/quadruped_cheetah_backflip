function [zeroCrossing,isterminal,direction] = back_take_off(t,X,p)

global FBy

%Tst = p.Tst;
%tTD = p.tTD;

% zeroCrossing =  t - tTD - Tst;
zeroCrossing =  FBy;
isterminal   =  1;
direction    =  -1;