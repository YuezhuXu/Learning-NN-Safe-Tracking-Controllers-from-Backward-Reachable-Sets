function [u_opt] = Dubin_Car_Control(x0,un,Ru,Tp)
m=length(un);
n=length(x0);
xp=center(Tp);
G=generators(Tp);
p=size(G,2);
f=[1,zeros(1,m),zeros(1,p)];
A1=[-ones(m,1),eye(m,m), zeros(m,p)];
B1=zeros(m,1);
A2=[-ones(m,1),-eye(m,m), zeros(m,p)];
B2=zeros(m,1);
A3=[zeros(m,1),eye(m,m), zeros(m,p)];
B3=un+Ru;
A4=[zeros(m,1),-eye(m,m), zeros(m,p)];
B4=-(un-Ru);
A5=[zeros(p,1),zeros(p,m), eye(p,p)];
B5=ones(p,1);
A6=[zeros(p,1),zeros(p,m), -eye(p,p)];
B6=ones(p,1);
A=[A1;A2;A3;A4;A5;A6];
B=[B1;B2;B3;B4;B5;B6];
Aeq=[zeros(n,1),G_d_Dubin_Car(x0),-G];
Beq=xp-F_d_Dubin_Car(x0);
options = optimoptions('linprog','Display','off');
X_opt=linprog(f,A,B,Aeq,Beq,[],[],options);
u_opt=X_opt(2:m+1);
end

