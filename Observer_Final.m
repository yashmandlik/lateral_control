A = [0 10;0 0];

B = [10;10];

C = [1 0];

D = 0;

%Check the controllability and observability

P = [B A*B];
if rank(P) == 2
    disp('System is Controllable')
else
    disp('System is not Controllable')
end

Q = [C ; C*A];
if rank(Q) == 2
    disp('System is Observable')
else
    disp('System is not Observable')
end

sys=ss(A,B,C,D)
Eigenvalues = eig(sys)

Poles = [-1 -10]; % Desired Poles

K = -acker(A,B,Poles) %State feedback controller gain matrix

l = acker(A',C',Poles) %State feedback observer

L = l'

