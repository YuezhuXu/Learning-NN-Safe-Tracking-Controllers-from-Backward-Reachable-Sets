clc
clear all
close all
figure

% random seed: 55 for rl, 66 for lu, 77 for ll

sd = 77;

if sd == 55
   name_str = 'rl';
elseif sd == 66
   name_str = 'lu';
elseif sd == 77
   name_str = 'll';
end

data_path = "optbased_ubounded/";
load(data_path+'Dubin_Car_Data_For_Plotting_'+num2str(sd)+"_"+name_str)

% plotting the map, BRSs, and the nominal trajectory.


%Xs=zonotope(Cs,Gs);
%plot(Xs,[1 2],'EdgeColor','none','FaceColor','g','FaceAlpha',.5)
hold on


NN=cell(numNodes+1,1);
for i=1:numNodes+1
NN{i}=zonotope(X(:,i),diag(Rx(:,i)));
%plot(NN{i},[1 2],'EdgeColor',0.8*[0.3010 0.7450 0.9330],'FaceColor',[0.3010 0.7450 0.9330],'FaceAlpha',0.1)
plot(NN{i},[1 2],'EdgeColor','none','FaceColor',[0.3010 0.7450 0.9330],'FaceAlpha',0.1)

hold on
end






Xt=zonotope(Ct,Gt);
plot(Xt,[1 2],'EdgeColor','none','FaceColor','b')


Xu=cell(1,Nu);

for i=1:Nu
Xu{i}=zonotope([CuArray(:,i),GuArray(:,:,i)]);
plot(Xu{i},[1 2],'EdgeColor','none','FaceColor','r')
end


for ind=numNodes+1:-1:1
plot(T{ind},[1 2],'EdgeColor',[0.3,0.3,0.3],'FaceColor',[0.5,0.5,0.5],'FaceAlpha',0.05,'linewidth',1)
%plot(T{ind},[1 2],'EdgeColor','none','FaceColor',[0.5,0.5,0.5],'FaceAlpha',0.2)

hold on
end
%plot(T{ind},[1 2],'EdgeColor',[0.3,0.3,0.3],'FaceColor',[0.5,0.5,0.5],'FaceAlpha',0.5)




xlabel('$x_{1}$','interpreter','latex')
ylabel('$x_{2}$','interpreter','latex')
set(gca,'fontsize',15)
set(gca,'ticklabelinterpreter','latex')
xlim([Xsl(1),Xsu(1)])
ylim([Xsl(2),Xsu(2)])
%grid on
box on

warning('Number of nodes is %d',numNodes)

