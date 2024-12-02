function [h_opt] = Optimizing_Station_Dubin_Car(X0,Rxm,u0,Rum,Gx,r)


n=3;
m=2;


Gi=abs(pinv(Gx));

  




hm=[Rxm;Rum];
[H1,H2,H3]=Hessian_Dubin_Car(X0,hm(1:n),u0,hm(n+1:n+m));
M=zeros(size(Gi,1),n+m);
for j=1:size(Gi,1)
M(j,:)=transpose(hm)*(Gi(j,1)*H1+Gi(j,2)*H2+Gi(j,3)*H3);  
end




Em=zeros(n,1);
Em(1)=0.5*dot(hm,H1*hm);
Em(2)=0.5*dot(hm,H2*hm);
Em(3)=0.5*dot(hm,H3*hm);

W0=Gi*Em;


%Formulating an LP that optimizes the station dimension;

f=zeros(1,n+m);
for i=1:n+m
    hh=hm;
    hh(i)=[];
f(i)=-prod(hh);
end 
A1=eye(n+m,n+m);
B1=hm;
A2=-eye(n+m,n+m);
B2=-r*hm;
A3=M;
B3=(r-1)*W0+M*hm;


A=[A1;A2;A3];
B=[B1;B2;B3];
options = optimoptions('linprog','Display','off');

h_opt=linprog(f,A,B,[],[],[],[],options);



end


