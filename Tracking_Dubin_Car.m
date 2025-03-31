clc
clear all
close all

% This MATLAB Code computes backward reachable sets along nominal
% trajectories for the Dubin's car model


% load the following file (Trajectory_Dubin_CarA.mat). 

% The  map has a rectngular safe set given by the set [Xsl,Xsu],
% rectangular unsafe sets of the form [Xul,Xuu], where Xul and Xuu
% are the columns of the arrays XulArray and XuuArray, respectively,
% and a rectangular target set [Xtl,Xtu].

% random seed: 55 for rl, 66 for lu, 77 for ll
sd = 55;
rng(sd)

if sd == 55
   name_str = 'rl';
elseif sd == 66
   name_str = 'lu';
elseif sd == 77
   name_str = 'll';
end
data_path = "optbased_ubounded/";
load(data_path+"Trajectory_Dubin_Car_"+num2str(sd)+"_"+name_str+".mat")






Nu=size(XulArray,2); % Nu is the number of obstacles
n=length(X0); % n is the system dimension (3 for the Dubin car model).


Cs=0.5*(Xsl+Xsu); % Cs is the center of the safe set
Ds=0.5*(Xsu-Xsl); % Ds is the vector radius of the safe set
Gs=diag(Ds); % Gs is the generator matrix corresponding to the safe set


Ct=0.5*(Xtl+Xtu); % Ct is the center of the target set
Dt=0.5*(Xtu-Xtl); % Dt is the vector radius of the target set
Gt=diag(Dt); % Gt is the generator matrix corresponding to the target set


CuArray=0.5*(XulArray+XuuArray); % CuArray is the array of  centers of the unsafe sets
DuArray=0.5*(XuuArray-XulArray);% DuArray is the array of  vector radii of the unsafe sets
GuArray=zeros(n,n,Nu);
GuiArray=zeros(n,n,Nu);

for j=1:Nu
GuArray(:,:,j)=diag(DuArray(:,j)); % GuArray is the array of  generator matrices of the unsafe sets
end




C_input=0.5*(Uu+Ul); % Center of the input set
m=length(C_input); % dimension of the input (2 for the dubin car)
D_input=0.5*(Uu-Ul); % Vector radius of the input set 






% % Trajectory_input is the array whose columns are the input values used to
% construct the nominal trajectory. Note that the dubin Cart dynamics are 
% of the form 
% x_{n+1}=F_d(x_{n})+G_d(x_{n})*u_{n}
% In this code, u_{n}=Trajectory_input(:,n) and x_{n}=X(:,n)

Size_Input=size(Trajectory_input);
numNodes=Size_Input(2); % This is the number of input values used in constructing the trajectory






% For the backward reach set (BRS) computations, I need to have 
%rectangular neighborhods of the nominal state/input pairs 
% for two reasons,
% (1) the state neighborthoods are defined such that they do not
%interesect with the obstacles and they are contained in the safe set,
%so intersecting BRSs with 
%these neighborhoods of nominal states will ensure safety
% the neighborhoods will be used to estimate remainder errors from
%linearization of the nonlinear dynamics. We need to linearize
%locally along the nominal trajectory before proceeding with BRS
%computations.


% Array of vector and  radii of state-input stations or neighborhoods
Rx=zeros(n,numNodes+1);
Ru=zeros(m,numNodes);


tic;

% Computing the radii values
alpha=0.99; 
Rx(:,numNodes+1)=Dt-abs(X(:,numNodes+1)-Ct);
for i=numNodes:-1:1
   Distances=Inf(n,Nu+1);
   Distances(:,Nu+1)=Ds-abs(X(:,i)-Cs);
   for j=1:Nu
   [dist,ind]=inf_distance_with_index(X(:,i),CuArray(:,j),DuArray(:,j));
   Distances(ind,j)=alpha*dist;   
   end
   
   for j=1:n
   Rx(j,i)=min(Distances(j,:));
   end

Ru(:,i)=D_input-abs(Trajectory_input(:,i)-C_input);
end
t_boxes=toc;
 




% T is the cell of arrays of BRSs (They will be zonotopes).
T=cell(numNodes+1,1);
Tp=cell(numNodes,1);
Rum=zeros(m,numNodes);
T{numNodes+1}=zonotope(X(:,numNodes+1),diag(Rx(:,numNodes+1))); % The final reach set coincides with the 
%the neighborhood of the final trajectoy point 
%(that neighborhood fits completely inside the target set)



% State independent disturbance
% time step 0.01
% er=[2e-4;2e-4;1e-3]; % This is a disturbance term added to the car dynamics
% time step 0.05
er=[1e-3;1e-3;5e-3]; % For 77, 66, 55
% More conservative when compute backward reachable sets, time step 0.05
% er = [3e-3;3e-3;1.5e-2]; % 77 is fine


% In our computations, the neighborhoods of state/input nominal pairs 
% will have to be scaled down to increase the sizes of the BRSs.
% Here I scale down the radii Rx and Ru incrementally using a heuristic 
% such that an "estimate" of the volume of the BRS is maximized (locally).

% The two parameters below are used in the scaling procedure.
D_alpha=diag([1,1,0.4,0.4,1]); % scaling matrix is prescribed
N_opt_max=1000; % maximum number of allowed scaling iterations



tic;

E=zeros(n,1); % E is a vector corresponding to remainder error
%(it will be redefined in reach iteration)

for i=numNodes:-1:1
% I print out the index of the BRS to see progress, you may want to comment
% that.
i

% These are the system matrices from linearization
A_k=A_Linearization_Dubin_Car(X(:,i),Trajectory_input(:,i));   
B_k=B_Linearization_Dubin_Car(X(:,i));





% Below I compute the BRS without applying any scaling

hb=[Rx(:,i);Ru(:,i)]; 
Rxb=hb(1:n); % Rx  after one scaling
Rub=hb(n+1:n+m); %Ru  after one scaling
           
           
% Hessian matrices are computed below 
%(They are functions of the neighborhoods)
[H1,H2,H3]=Hessian_Dubin_Car(X(:,i),Rxb,Trajectory_input(:,i),Rub);  
          
          
% Estimating the remainder errors
E(1)=0.5*dot(hb,H1*hb);
E(2)=0.5*dot(hb,H2*hb);
E(3)=0.5*dot(hb,H3*hb);
          
Gx=generators(T{i+1});
Gi=pinv(Gx);


  
% Below I compute the generators matrix MD associated with the Minkowski
% difference T{i+1}-W
 [MD,empt_check]=Zonotopic_Minkowski_Difference_BRS(generators(T{i+1}),E+er,Gi);

 
  % Here I check if the Minkowski difference is empty
  if empt_check==1
     Vt=0; % the size estimate of the BRS is set to be zero if the Minkowski difference is empty
  else


% Below, I compute the actual BRS T{i} (the intersetion step is embedded in the computation below)
T{i}=Zonotopic_Intersection_Improved(X(:,i),A_k\[MD,B_k*diag(Rub)],Rxb);

% Vt is a size estimate of the BRS. 
 Vt=1/norm(pinv(generators(T{i})),inf); 
  end



Vmax=Vt;







% % Below is the iterative scaling process
Vmax=Vt;
Rum(:,i)=Rub;
indmax=0;
ind_opt=0;


%In the following loop, I proceed with the scaling computations until a local maximum is 
% acheived or until the maximum number of iterations is reached



while  ind_opt<=N_opt_max
    ind_opt=ind_opt+1;

           hb=D_alpha*hb;
           Rxb=hb(1:n); % Rx  after one scaling
           Rub=hb(n+1:n+m); %Ru  after one scaling
           
           
           % Hessian matrices are computed below 
           %(They are functions of the neighborhoods)
          [H1,H2,H3]=Hessian_Dubin_Car(X(:,i),Rxb,Trajectory_input(:,i),Rub);  
          
          
          % Estimating the remainder errors
          E(1)=0.5*dot(hb,H1*hb);
          E(2)=0.5*dot(hb,H2*hb);
          E(3)=0.5*dot(hb,H3*hb);
          
        
        % Below I compute the generators matrix MD associated with the Minkowski
        % difference T{i+1}-W  
         [MD,empt_check]=Zonotopic_Minkowski_Difference_BRS(generators(T{i+1}),E+er,Gi);

         % Here I check if the Minkowski difference is empty
         if empt_check==1
            Vt=0;
         else

        
         
         %Xbrs below is the the BRS, where intersection is with the neighborhood
         %[X(:,i)-Rx(:,i),X(:,i)+Rx(:,i)]. 
         Xbrs=Zonotopic_Intersection_Improved(X(:,i),A_k\[MD,B_k*diag(Rub)],Rxb);
         
         % Vt is the size estimate of the BRS. 
          Vt=1/norm(pinv(generators(Xbrs)),inf); 

         end

         % In the following  if statement, I try to
         % find the number of scalings such that the size estimate
         % Vt reaches its local maximum Vmax.
         if Vt>=Vmax
            Vmax=Vt;
            indmax=ind_opt;
            T{i}=Xbrs;
            Rum(:,i)=Rub;

         else  % this corresonds to passing by the local maximum volume
             indmax=ind_opt-1;
             break;
         end
end
     
% The set below is T{i+1}-W, which is used in control synthesis
Tp{i}=zonotope(X(:,i+1),Zonotopic_Minkowski_Difference_BRS(generators(T{i+1}),er,Gi));        

% The if statement below checks if the iterative scaling procedure failed.
 if Vmax==0
   warning('Iterations Failed failed!!');
   break;
 end  




% Order reduction of BRS to save memory 

T{i}=reduceUnderApprox(T{i},'sum',30);


end 



t_tracking=toc;






% The data are plotted in the code Plotting_Dubin_Car
save(data_path+"Dubin_Car_Data_For_Plotting_"+num2str(sd)+"_"+name_str+".mat","numNodes","Nu","Cs","Gs","Xsl","Xsu","X","Trajectory_input","Rx","Rum","Ct","Gt","CuArray","GuArray","T","Tp")




  
  