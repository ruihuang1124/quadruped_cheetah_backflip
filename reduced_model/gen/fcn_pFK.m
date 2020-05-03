function [pFK] = fcn_pFK(q,p)

pFK = zeros(2,1);

  pFK(1,1)=q(1) + (p(2)*cos(q(3)))/2 + p(3)*cos(q(4))*sin(q(3)) + p(3)*cos(q(3))*sin(q(4));
  pFK(2,1)=q(2) + (p(2)*sin(q(3)))/2 - p(3)*cos(q(4))*cos(q(3)) + p(3)*sin(q(4))*sin(q(3));

 