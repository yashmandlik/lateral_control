%Non linear, variable gain controller implementation

%Sim script based on error dynamics
clear;
close all

%The following vector has the 
x0=[0;1;37;0];
xk=x0;

t0=0;
tk=t0;
dt=0.001;

V0=25; %close to the one from the trajectory

tf=6;

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
%Place constraints on input
u1_constraints=[-11.787 11.787]; %The max acceleration corresponds to this:https://www.engadget.com/2017/02/07/tesla-model-s-ludicrous-acceleration-record/

u2_constraints=[-1 1]; %Arbitrary choice, representing the tangents of steering angles

p=[-2+i;-2-i;-20;-21.5]; %first two eigenvalues are dominant

xref=[];
e=[];

G=lqr(A,B,eye(4),eye(2));

error_integral=[0;0;0;0];

K=place(A,B,p); %State Feedback using pole placement
uref=[];

for i=1:nsteps
    
    %keyboard();
    xref(:,i)=desired_state(tk);
    uref(:,i)=reference_input(tk);
    
    %Hypothetical uref
    %uref=[0;pi/20*sin(pi/20*tk)/V0];
    
    %keyboard();
    e(:,i)=xref(:,i)-xk(:,i);
    
    %The following implements a fixed gain controller (Obtained from PID or
    %LQR)
    %uk(:,i)=uref(:,i)+G*(xref(:,i)-xk(:,i));
    
    %Now we linearize about each point and apply a linear controller in the following subsection
    [At,Bt]=linearize_model_at(xref(:,i));
    Gt=lqr(At,Bt,eye(4),eye(2));
    
    %Now implement the controller based on pointwise linearization
    uk(:,i)=uref(:,i)+Gt*(xref(:,i)-xk(:,i));  
    
    %Uncomment the following line for constrained inputs
    %uk(:,i)=constrain_input(uk(:,i),u1_constraints,u2_constraints);
   
    
    %keyboard();
    [t,x]=ode45(@(t,x) dxdt(x,uk(:,i)),[tk tk+dt],xk(:,i));
    
    xk(:,i+1)=x(end,:)';%xk(:,i)+dxdt(xk(:,i),uk)*dt
    
    tk=tk+dt;
end

%Time for plots
figure(1)
plot(xref(1,:),xref(2,:))
hold on
plot(xk(1,:),xk(2,:))
legend('reference trajectory','actual trajectory')
axis equal

