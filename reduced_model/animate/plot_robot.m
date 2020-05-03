function  plot_robot(X,pFtoe,pBtoe,p)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

params = p.params;
q = X(1:3);
qdot = X(4:6);

pFH = fcn_pFH(q,params);
pBH = fcn_pBH(q,params);

chain = [pBH pFH];

plot(chain(1,:),chain(2,:),'k','LineWidth',3);
hold on
plot(chain(1,:),chain(2,:),'.r','LineWidth',4);

ne = 5; a = 1; ro = 0.01;
[xs, ys] = spring(pFH(1),pFH(2),pFtoe(1),pFtoe(2),ne,a,ro);
plot(xs,ys,'LineWidth',2)

[xs, ys] = spring(pBH(1),pBH(2),pBtoe(1),pBtoe(2),ne,a,ro);
plot(xs,ys,'LineWidth',2)

axis equal
grid on
xlabel('X_0 [m]')
ylabel('Y_0 [m]')
xlim([-1, 1])
ylim([-1, 1])

hold off

end

