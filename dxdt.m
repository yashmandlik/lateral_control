function [xdot]=dxdt(x,u)
    %Calculates the state derivative
    %keyboard();
    x0=x(1);y=x(2);V=x(3);theta=x(4);
    %theta is in radians
    
    xdot=[0;0;0;0];
    xdot(1)=V*cos(theta);
    xdot(2)=V*sin(theta);
    xdot(3)=u(1); 
    xdot(4)=V*u(2);