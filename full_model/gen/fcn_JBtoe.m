function [JBtoe] = fcn_JBtoe(q,p)

JBtoe = zeros(2,7);

  JBtoe(1,1)=1;
  JBtoe(1,2)=0;
  JBtoe(1,3)=(p(2)*sin(q(3)))/2 + p(4)*cos(q(7))*(cos(q(6))*cos(q(3)) - sin(q(6))*sin(q(3))) - p(4)*...
         sin(q(7))*(cos(q(6))*sin(q(3)) + cos(q(3))*sin(q(6))) + p(3)*cos(q(6))*cos(q(3)) - p(3)*sin(q(6))*sin(q(3));
  JBtoe(1,4)=0;
  JBtoe(1,5)=0;
  JBtoe(1,6)=p(4)*cos(q(7))*(cos(q(6))*cos(q(3)) - sin(q(6))*sin(q(3))) - p(4)*sin(q(7))*(cos(q(6))*...
         sin(q(3)) + cos(q(3))*sin(q(6))) + p(3)*cos(q(6))*cos(q(3)) - p(3)*sin(q(6))*sin(q(3));
  JBtoe(1,7)=p(4)*cos(q(7))*(cos(q(6))*cos(q(3)) - sin(q(6))*sin(q(3))) - p(4)*sin(q(7))*(cos(q(6))*...
         sin(q(3)) + cos(q(3))*sin(q(6)));
  JBtoe(2,1)=0;
  JBtoe(2,2)=1;
  JBtoe(2,3)=p(4)*cos(q(7))*(cos(q(6))*sin(q(3)) + cos(q(3))*sin(q(6))) - (p(2)*cos(q(3)))/2 + p(4)*...
         sin(q(7))*(cos(q(6))*cos(q(3)) - sin(q(6))*sin(q(3))) + p(3)*cos(q(6))*sin(q(3)) + p(3)*cos(q(3))*sin(q(6));
  JBtoe(2,4)=0;
  JBtoe(2,5)=0;
  JBtoe(2,6)=p(4)*cos(q(7))*(cos(q(6))*sin(q(3)) + cos(q(3))*sin(q(6))) + p(4)*sin(q(7))*(cos(q(6))*...
         cos(q(3)) - sin(q(6))*sin(q(3))) + p(3)*cos(q(6))*sin(q(3)) + p(3)*cos(q(3))*sin(q(6));
  JBtoe(2,7)=p(4)*cos(q(7))*(cos(q(6))*sin(q(3)) + cos(q(3))*sin(q(6))) + p(4)*sin(q(7))*(cos(q(6))*...
         cos(q(3)) - sin(q(6))*sin(q(3)));

 