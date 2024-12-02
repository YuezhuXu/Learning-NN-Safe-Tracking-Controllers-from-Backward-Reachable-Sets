% This code is for RRT path planning of the Dubin's car system
% of the form x+=f(x)+g(x)*u with safe set (box) Xs,
% unsafe region (union of boxes) Xu, initial point X0 in Xs, 
% and target set (box) Xt. A robustness margin delta is used to inflate and
% deflate the unsafe and safe sets, respectively.

clc
clear all
close all

data_path = "optbased_ubounded/";
% random seed: 55 for rl, 66 for lu, 77 for ll
sd = 77; 
rng(sd)


if sd == 55
   name_str = 'rl';
elseif sd == 66
   name_str = 'lu';
elseif sd == 77
   name_str = 'll';
end

% system dimension 
n=3;
m=2;

% maximum number of iterations
N_iter=300;  

N_extend=40;

% maximum number of vertices in RRT plans
Nv=N_iter*(1+N_extend);


% robustness margins
delta_vector=[0.1;0.1;pi/10];
% delta_vector=[0.15;0.15;pi/8];




% safe set Xs

Xsl=[0;0;-pi/2];

Xsu=[5;5;pi/2];


Cs=0.5*(Xsl+Xsu);
Ds=0.5*(Xsu-Xsl)-delta_vector;
Gs=diag(Ds);


% Initial Condition
if sd == 55
   X0=[4;0.5;-pi/4]; % rl case, sd = 55
elseif sd == 66
   X0=[0.5;4.5;-pi/4]; % lu case, sd = 66
elseif sd == 77
   X0=[1;1;0]; % ll case, sd = 77
end







sX=size(X0);

% system dimension
n=sX(1);


% Target set Xt 

Xtl=[3.5;4.5;-pi/5];
Xtu=[5;5;pi/5];
Ct=0.5*(Xtl+Xtu);
Dt=0.5*(Xtu-Xtl)-delta_vector;
Gt=diag(Dt);


% Unsafe set Xu
XulArray=[1.5;1;-pi/2];
XuuArray=[3;2;pi/2];
XulArray=[XulArray, [1.5;3.5;-pi/2]];
XuuArray=[XuuArray,[2.5;5;pi/2]];
XulArray=[XulArray, [3.5;2;-pi/2]];
XuuArray=[XuuArray,[4.5;3.5;pi/2]];

sXu=size(XulArray);
Nu=sXu(2);

CuArray=0.5*(XulArray+XuuArray);
DuArray=0.5*(XuuArray-XulArray)+delta_vector;






% control input set U
% time step 0.05   
uxl=-0.4;
uxu=0.4;

uyl=-0.25; 
uyu=0.25; 
epsu_vector=[0.01;0.01]; % parameter for deflating input set

Ul=[uxl;uyl];

Uu=[uxu;uyu];
%input dimension
sizeU=size(Uu);
nu=sizeU(1);

c_input=0.5*(Ul+Uu);







%initializing the RRT Tree and associated tree structures
Tree=zeros(n,Nv);
F_d_Tree=zeros(n,Nv);
G_d_Tree=zeros(n,nu,Nv);
Safety_Tree=zeros(1,Nv);
Nodes=ones(1,Nv);
Input_Array=zeros(nu,Nv);
Images=ones(1,Nv);



ind=1;
Tree(:,1)=X0; % adding initial point to the tree.
F_d_Tree(:,1)=F_d_Dubin_Car(Tree(:,1));
G_d_Tree(:,:,1)=G_d_Dubin_Car(Tree(:,1));
dist_T=300;










while  ind<Nv 
    ind
    
 % generating a random sample 
 if ind<0.75*Nv
 x_sample=Xsl+(Xsu-Xsl).*rand(n,1);
 else
 x_sample=Xtl+(Xtu-Xtl).*rand(n,1);   
 end
 
 % Generating random Input value
 u_sample=(Ul+epsu_vector)+((Uu-epsu_vector)-(Ul+epsu_vector)).*rand(nu,1);
 
 dist=100*norm(Xsu-Xsl,inf);
  for ind_dist=1:ind
      xt=F_d_Tree(:,ind_dist)+G_d_Tree(:,:,ind_dist)*u_sample;
      check=Safety_Check(xt,Cs,Ds,CuArray,DuArray);
       if check==1
          dr=norm(x_sample-xt,inf); 
       else
          dr=Inf;
       end
     if dr<=dist
         ind_near=ind_dist;
        dist=dr;
     end
  end
 
 x_near=Tree(:,ind_near);
 
 
 x_new=F_d_Tree(:,ind_near)+G_d_Tree(:,:,ind_near)*u_sample;
 ind=ind+1;
 Tree(:,ind)=x_new;
 F_d_Tree(:,ind)=F_d_Dubin_Car(x_new);
 G_d_Tree(:,:,ind)=G_d_Dubin_Car(x_new);
 Nodes(ind-1)=ind_near;
 Input_Array(:,ind-1)=u_sample;
 Images(ind-1)=ind;
 dist_T=norm(Gt\(x_new-Ct),inf);
 

 if dist_T<=1
 break;
 end
 


% extending step
x_new_extend_p=x_new;

for ind_extend=1:N_extend
x_new_extend=F_d_Dubin_Car(x_new_extend_p)+G_d_Dubin_Car(x_new_extend_p)*u_sample;
safetycheck=Safety_Check(x_new_extend,Cs,Ds,CuArray,DuArray); 
if safetycheck==1
 ind=ind+1;
 Tree(:,ind)=x_new_extend;
 F_d_Tree(:,ind)=F_d_Dubin_Car(x_new_extend);
 G_d_Tree(:,:,ind)=G_d_Dubin_Car(x_new_extend);
 Nodes(ind-1)=ind-1;
  Input_Array(:,ind-1)=u_sample;
 Images(ind-1)=ind;
 dist_T=norm(Gt\(x_new_extend-Ct),inf);
 x_new_extend_p=x_new_extend;
else
    break;
end

if dist_T<=1
 break;
end
 
end

    
end    
    
    







 
 
  if dist_T<=1
   % warning('Problem solved!')   
   G = digraph(Nodes,Images);
  Path=shortestpath(G,1,ind);
  Trajectory_input=zeros(nu,numel(Path)-1);
  X=zeros(n,numel(Path));
 
  
  
  X(:,1)=X0;
Ind_List=zeros(1,numel(Path)-1);
for j=numel(Ind_List):-1:1
    Ind_List(j)=Path(j+1)-1;
end
  
  for j=1:numel(Ind_List)
    u=Input_Array(:,Ind_List(j));
    X(:,j+1)=F_Dubin_Car(X(:,j),u);
    Trajectory_input(:,j)=u;
  end
  save(data_path+"Trajectory_Dubin_Car_"+num2str(sd)+"_"+name_str+".mat","X","Trajectory_input","X0","Xsl","Xsu","Xtl","Xtu","XulArray","XuuArray","Ul","Uu");  
  else 
  end





Xs=zonotope([0.5*(Xsl+Xsu),diag(0.5*(Xsu-Xsl))]);
plot(Xs,[1 2],'g','linewidth',2)
hold on
Xt=zonotope([0.5*(Xtl+Xtu),diag(0.5*(Xtu-Xtl))]);
plot(Xt,[1 2],'b','linewidth',2)

Xu=cell(1,Nu);

for i=1:Nu
Xu{i}=zonotope([CuArray(:,i),diag(0.5*(XuuArray(:,i)-XulArray(:,i)))]);
plot(Xu{i},[1 2],'r','linewidth',2)
end


scatter(Tree(1,1:ind),Tree(2,1:ind),'g')
if dist_T<=1
plot(X(1,:),X(2,:),':ob','linewidth',2) 
end



xlim([Xsl(1),Xsu(1)])
ylim([Xsl(2),Xsu(2)])


