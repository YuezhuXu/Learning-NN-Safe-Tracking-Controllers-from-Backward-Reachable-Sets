function [G_s,empt_check] = Zonotopic_Minkowski_Difference_BRS(G,L,Gi)


S1=size(G);
p=S1(2);


K=abs(Gi)*L;

Gt=zeros(p,p);

for i=1:p
   if K(i)<=1
   Gt(i,i)=1-K(i);
    empt_check=0;
   else
       Gt=[];
       empt_check=1;
       break;
   end
end


if empt_check==0
G_s=G*Gt;
else 
   G_s=[];
end

