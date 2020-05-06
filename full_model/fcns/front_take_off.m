function [zeroCrossing,isterminal,direction] = front_take_off(t,X,p)

global FFy

%Tst = p.Tst;
%tTD = p.tTD;

% zeroCrossing =  t - tTD - Tst;
zeroCrossing =  FFy;
isterminal   =  1;
direction    =  -1;