A = [0 10;0 0];
B = [10;10];
C = [1 0];
D = 0;

sys = ss(A,B,C,D);

L = acker(A',C',[-1 -10])';

A1 = [A zeros(2);L*C A-L*C];
B1 = [B;B];
C1 = [C 0 0;0 0 C];
D1 = 0;

sys1 = ss(A1,B1,C1,D1);

t = 0:1e-2:39.31;
y = lsim(sys1,out.u,t);
plot(t,y(:,1),t,y(:,2))
legend('plant output','observer output estimate')