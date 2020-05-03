function  plot_robot(X,p,r0F,r0B)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

params = p.params;
q = X(1:7);
qdot = X(8:14);

pFH = fcn_pFH(q,params);
pBH = fcn_pBH(q,params);
pFK = fcn_pFK(q,params);
pBK = fcn_pBK(q,params);
pFtoe = fcn_pFtoe(q,params);
pBtoe = fcn_pBtoe(q,params);

chain = [pBtoe pBK pBH pFH pFK pFtoe];

plot(chain(1,:),chain(2,:),'k','LineWidth',3);
hold on
plot(chain(1,:),chain(2,:),'.r','LineWidth',4);
axis equal
grid on
xlabel('X_0 [m]')
ylabel('Y_0 [m]')
xlim([-1, 1])
ylim([-1, 1])

end

