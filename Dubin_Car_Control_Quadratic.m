function [u_opt] = Dubin_Car_Control_Quadratic(x0,un,Ru,Tp)
m=length(un);
n=length(x0);
xp=center(Tp);
G=generators(Tp);
p=size(G,2);

H=[eye(m,m),zeros(m,p);zeros(p,m),eye(p,p)];

f=[zeros(1,m),zeros(1,p)];
A1=[];%[-ones(m,1),eye(m,m), zeros(m,p)];
B1=[];%zeros(m,1);
A2=[];%[-ones(m,1),-eye(m,m), zeros(m,p)];
B2=[];%zeros(m,1);
A3=[eye(m,m), zeros(m,p)];
B3=un+Ru;
A4=[-eye(m,m), zeros(m,p)];
B4=-(un-Ru);
A5=[zeros(p,m), eye(p,p)];
B5=ones(p,1);
A6=[zeros(p,m), -eye(p,p)];
B6=ones(p,1);
A=[A1;A2;A3;A4;A5;A6];
B=[B1;B2;B3;B4;B5;B6];
Aeq=[G_d_Dubin_Car(x0),-G];
Beq=xp-F_d_Dubin_Car(x0);
%options = optimoptions('linprog','Display','off');
%X_opt=linprog(f,A,B,Aeq,Beq,[],[],options);
% options = optimoptions('quadprog','Display','off');
% options = optimoptions('quadprog','Display','off','OptimalityTolerance',1e-10,'StepTolerance',1e-10, 'MaxIterations',1e10);
   
x0=[un;zeros(p,1)];
X_opt = quadprog(H,f,A,B,Aeq,Beq,[],[],[]); %,options);
u_opt=X_opt(1:m);
end

