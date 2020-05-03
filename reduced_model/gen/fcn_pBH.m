function [pBH] = fcn_pBH(q,p)

pBH = zeros(2,1);

  pBH(1,1)=q(1) - (p(2)*cos(q(3)))/2;
  pBH(2,1)=q(2) - (p(2)*sin(q(3)))/2;

 