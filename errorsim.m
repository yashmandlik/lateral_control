%Sim script based on error dynamics
clear;
x0=[-100;0.01;5;0.1*pi/180];
xk=x0

t0=0;
tk=t0;
dt=0.001;

V0=25; %close to the one from the trajectory

tf=100;

nsteps=1+(tf-t0)/dt;

%The following matrices represent model after linearization
A=[0,0,1,0;
    0,0,0,V0;
    0,0,0,0;
    0,0,0,0];
B=[0,0;
    0,0;
    ;1,0;
    0,V0]; 

%The first input is the pedal while the 2nd is the wheel


p=[-0.002;-0.0030;-6;-4]; %first two eigenvalues are dominant

xref=[];
e=[];

G=lqr(A,B,eye(4),eye(2));

error_integral=[0;0;0;0];

K=place(A,B,p);

for i=1:nsteps
    
    
    K;
    
    xref(:,i)=desired_state(tk);
    uref=reference_input(tk);
    
    %Hypothetical uref
    uref=[0;pi/20*sin(pi/20*tk)/V0];
    
    %keyboard();
    e(:,i)=xref(:,i)-xk(:,i);
    error_integral=error_integral+e(:,i)*dt;
    uk=uref+G*(xref(:,i)-xk(:,i));%+K*error_integral;% u=uref+K*error
    
    %keyboard();
    [t,x]=ode45(@(t,x) dxdt(x,uk),[tk tk+dt],xk(:,i));
    
    xk(:,i+1)=x(end,:)';%xk(:,i)+dxdt(xk(:,i),uk)*dt
    
    tk=tk+dt;
end