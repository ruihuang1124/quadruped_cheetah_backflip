function [pBK] = fcn_pBK(q,p)

pBK = zeros(2,1);

  pBK(1,1)=q(1) - (p(2)*cos(q(3)))/2 + p(3)*cos(q(6))*sin(q(3)) + p(3)*cos(q(3))*sin(q(6));
  pBK(2,1)=q(2) - (p(2)*sin(q(3)))/2 - p(3)*cos(q(6))*cos(q(3)) + p(3)*sin(q(6))*sin(q(3));

 