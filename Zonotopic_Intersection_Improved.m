function [Z_int] = Zonotopic_Intersection_Improved(c,G,rb)

%Here, we assume the intersection to have the form c+G(ct+dt*B+pinv(G)*dt*B)
LPopts = optimoptions('linprog','Display','off','ConstraintTolerance',1e-9);
S=size(G);
n=S(1);
p=S(2);
Gi=pinv(G);
%alpha_m=min([0.2*min(rb)/norm(G,inf),1]);
alpha_m=min([0.1*min(rb)/norm(G,inf),1]);

f=-ones(1,p+n); 
A1=[-eye(p,p),zeros(p,n)];
B1=-alpha_m*ones(p,1);
A2=[zeros(n,p),-eye(n,n)];
B2=zeros(n,1);
A3=[eye(p,p),abs(Gi)];
B3=ones(p,1);
A4=[abs(G), eye(n,n)];
B4=rb;
A=[A1;A2;A3;A4];
B=[B1;B2;B3;B4];
X_opt=linprog(f,A,B,[],[],[],[],LPopts);
d_v=X_opt(1:p);
d_t_v=X_opt(p+1:end);
Z_int=zonotope(c,[G*diag(d_v),diag(d_t_v)]);
end

