function myplot1(x,y,t)

%     t = rand(1,10) ; 
%     x = rand(1,10) ; 
%     y = rand(1,10) ;
    h = plot3(t(1),x(1),y(1)) ;
    axis([min(t) max(t) min(x) max(x) min(y) max(y)])
    for i = 1:length(t)
        set(h,'Xdata',t(1:i),'Ydata',x(1:i),'Zdata',y(1:i)) ;
        drawnow
        pause(0.5)
    end