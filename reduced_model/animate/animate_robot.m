function animate_robot(tin,Xin,pFtoein,pBtoein,FFspringin,FBspringin,p)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

global pFtoe0 pBtoe0
N = 50;
[t,X] = even_sample(tin,Xin,N);
[t,pFtoe] = even_sample(tin,pFtoein,N);
[t,pBtoe] = even_sample(tin,pBtoein,N);
[t,FFspring] = even_sample(tin,FFspringin,N);
[t,FBspring] = even_sample(tin,FBspringin,N);
nt = length(t);


f = figure;
v = VideoWriter('video.avi');
open(v)
set(f, 'doublebuffer', 'on');

for ii = 1:nt
    X_ = X(ii,:)';
    pFtoe_ = pFtoe(ii,:)';
    pBtoe_ = pBtoe(ii,:)';
    set(gcf, 'Position',  [100, 100, 1000, 600])
    subplot(2,3,[1,2,4,5])
    plot_robot(X_,pFtoe_,pBtoe_,p);
    title(['Time =' num2str(t(ii),'%6.2f') 's'])
    pause(0.0001)
    
    subplot(2,3,3)
    plot(t(1:ii),FFspring(1:ii))
    title(['Front spring force'])
    
    subplot(2,3,6)
    plot(t(1:ii),FBspring(1:ii))
    title(['Back spring force'])
    
    F = getframe(f);
    drawnow
    writeVideo(v,F)
end

end

