function animate_robot(tin,Xin,p)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

N = 100;
[t,X] = even_sample(tin,Xin,N);
nt = length(t);

f = figure;
v = VideoWriter('video.avi');
open(v)
set(f, 'doublebuffer', 'on');

for ii = 1:nt
    X_ = X(ii,:)';
    set(gcf, 'Position',  [100, 100, 1000, 600])
    subplot(2,3,[1,2,4,5])
    plot_robot(X_,p);
    title(['Time =' num2str(t(ii),'%6.2f') 's'])
    pause(0.0001)
    
    
    F = getframe(f);
    drawnow
    writeVideo(v,F)
end

end