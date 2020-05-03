function [JFtoe] = fcn_JFtoe(q,p)

JFtoe = zeros(2,7);

  JFtoe(1,1)=1;
  JFtoe(1,2)=0;
  JFtoe(1,3)=p(4)*cos(q(5))*(cos(q(4))*cos(q(3)) - sin(q(4))*sin(q(3))) - (p(2)*sin(q(3)))/2 - p(4)*...
         sin(q(5))*(cos(q(4))*sin(q(3)) + cos(q(3))*sin(q(4))) + p(3)*cos(q(4))*cos(q(3)) - p(3)*sin(q(4))*sin(q(3));
  JFtoe(1,4)=p(4)*cos(q(5))*(cos(q(4))*cos(q(3)) - sin(q(4))*sin(q(3))) - p(4)*sin(q(5))*(cos(q(4))*...
         sin(q(3)) + cos(q(3))*sin(q(4))) + p(3)*cos(q(4))*cos(q(3)) - p(3)*sin(q(4))*sin(q(3));
  JFtoe(1,5)=p(4)*cos(q(5))*(cos(q(4))*cos(q(3)) - sin(q(4))*sin(q(3))) - p(4)*sin(q(5))*(cos(q(4))*...
         sin(q(3)) + cos(q(3))*sin(q(4)));
  JFtoe(1,6)=0;
  JFtoe(1,7)=0;
  JFtoe(2,1)=0;
  JFtoe(2,2)=1;
  JFtoe(2,3)=(p(2)*cos(q(3)))/2 + p(4)*cos(q(5))*(cos(q(4))*sin(q(3)) + cos(q(3))*sin(q(4))) + p(4)*...
         sin(q(5))*(cos(q(4))*cos(q(3)) - sin(q(4))*sin(q(3))) + p(3)*cos(q(4))*sin(q(3)) + p(3)*cos(q(3))*sin(q(4));
  JFtoe(2,4)=p(4)*cos(q(5))*(cos(q(4))*sin(q(3)) + cos(q(3))*sin(q(4))) + p(4)*sin(q(5))*(cos(q(4))*...
         cos(q(3)) - sin(q(4))*sin(q(3))) + p(3)*cos(q(4))*sin(q(3)) + p(3)*cos(q(3))*sin(q(4));
  JFtoe(2,5)=p(4)*cos(q(5))*(cos(q(4))*sin(q(3)) + cos(q(3))*sin(q(4))) + p(4)*sin(q(5))*(cos(q(4))*...
         cos(q(3)) - sin(q(4))*sin(q(3)));
  JFtoe(2,6)=0;
  JFtoe(2,7)=0;

 