function [zeroCrossing,isterminal,direction] = take_off(t,X,p)

global FBy
global FFy

%Tst = p.Tst;
%tTD = p.tTD;
if FFy <= FBy
    zeroCrossing = FFy;
else
    zeroCrossing = FBy;
end

% zeroCrossing =  t - tTD - Tst;
% zeroCrossing =  FBy;
isterminal   =  1;
direction    =  -1;