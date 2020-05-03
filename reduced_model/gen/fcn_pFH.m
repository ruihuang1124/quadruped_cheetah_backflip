function [pFH] = fcn_pFH(q,p)

pFH = zeros(2,1);

  pFH(1,1)=q(1) + (p(2)*cos(q(3)))/2;
  pFH(2,1)=q(2) + (p(2)*sin(q(3)))/2;

 