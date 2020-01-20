function [uref] = reference_input(t)
%Calculates reference input using numerical differentiation
dt=0.001;

xref1=desired_state(t);

xref2=desired_state(t+dt);

xdot_ref=(xref2-xref1)/dt;

V0=xref1(3);

u1=xdot_ref(3);
u2=xdot_ref(4)/V0; %Refer to the model equations (Each system parameter is assumed to be unity)

uref=[u1;u2];
end

