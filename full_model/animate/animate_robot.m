function animate_robot(tin,Xin,uin,GRFFin,GRFBin,p)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

N = 100;
[t,X] = even_sample(tin,Xin,N);
[t,uin] = even_sample(tin,uin,N);
[t,GRFFin] = even_sample(tin,GRFFin,N);
[t,GRFBin] = even_sample(tin,GRFBin,N);
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
    
    subplot(2,3,3)
    plot(t(1:ii),uin(1:ii,:))
    title(['Torque profile'])
    legend({'taufh','taufk','taubh','taubk'},'Location','southeast')
    
    subplot(2,3,6)
    plot(t(1:ii),GRFFin(1:ii,:))
    hold on
    plot(t(1:ii),GRFBin(1:ii,:))
    title(['Force profile'])
    legend({'GRFFx','GRFFy', 'GRFBx', 'GRFBy'},'Location','northeast')
    hold off
    
    F = getframe(f);
    drawnow
    writeVideo(v,F)
end

end