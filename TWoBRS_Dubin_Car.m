clc
clear all
close all

% random seed: 55 for rl, 66 for lu, 77 for ll
sd = 55;
if sd == 55
   name_str = 'rl';
elseif sd == 66
   name_str = 'lu';
elseif sd == 77
   name_str = 'll';
end
rng(123)



%
data_path = "optbased_ubounded/";
load(data_path+'Dubin_Car_Data_For_Plotting_'+num2str(sd)+"_"+name_str)


% Get cu,Ru, sampled states xu
% In terms of time step, T{N+1} corresponds to Tp{N} (Tp{N} <--> T{N+1})
% We want any sampling points in T{N} to be driven inside Tp{N} in the next step.
% use the last 2 zonotopes
indices = 1:numNodes; 
% [100,40] for 70 works
N_Sample_ex = 60;
N_Sample_uni = 40;


states_starting = randPoint(T{1}, N_Sample_uni, "uniform");
for N = 1:numNodes % indices
    disp(N)
    % T{N} --> Tp{N}
    cu = Trajectory_input(:, N);
    Ru = Rum(:, N);
    c1 = center(T{N});
    
    % states_uni = randPoint(T{N}, N_Sample_uni, "uniform");
    % For sampling manually
    G = T{N}.generators;
    n = size(G,2);
    r = -1+2.*rand(n,N_Sample_uni);
    states_uni = c1+G*r;
    
    % T_reduced = zonotope(T{N}.c, T{N}.G(:, 1:min(size(T{N}.G, 2), 40)));  % Use only the first 10 generators
    % states_uni = randPoint(T_reduced, N_Sample_uni, "uniform:hitAndRun");
    % scale_factor = max(abs(G(:)));  % Normalize by largest absolute value
    % G = G / scale_factor;            % Rescale
    % T_scaled = zonotope(c1 / scale_factor, G);
    % states_uni = randPoint(T_scaled, N_Sample_uni, "uniform:hitAndRun");
    
    % values = [-1,0,1];
    % r_bd = values(randi([1, 3], n, N_Sample_bd));% 2 .* randi([0, 1], n, N_Sample_bd) - 1;
    % states_bd = c1+G*r_bd;

    
    %states_rad = randPoint(T{N}, N_Sample_rad, "radius");
    %states_gaus = randPoint(T{N}, N_Sample_gaus, "gaussian");
    
    c2 = center(Tp{N});
    states_ex = randPoint(T{N}, N_Sample_ex, "extreme");
    % states_bd = randPoint(T{N}, N_Sample_ex, "boundary");


    G2 = Tp{N}.generators;
    
    save(data_path+"\\sampling\\states_sampled_extreme_z"+num2str(N)+"_rand"+num2str(sd)+"_"+name_str+".mat", 'cu', 'Ru', 'states_ex', 'c1', 'c2', 'G2');
    % save(data_path+"states_sampled_radius_z"+num2str(N)+"_rand"+num2str(sd)+".mat", 'cu', 'Ru', 'states_rad', 'c1', 'c2', 'G2');
    % save(data_path+"states_sampled_gaussian_z"+num2str(N)+"_rand"+num2str(sd)+".mat", 'cu', 'Ru', 'states_gaus', 'c1', 'c2', 'G2');
    save(data_path+"\\sampling\\states_sampled_uniform_z"+num2str(N)+"_rand"+num2str(sd)+"_"+name_str+".mat", 'cu', 'Ru', 'states_uni', 'c1', 'c2', 'G2');
    % save(data_path+"states_sampled_boundary_z"+num2str(N)+"_rand"+num2str(sd)+".mat", 'cu', 'Ru', 'states_bd', 'c1', 'c2', 'G2');
end

%% Sample states and Derive Optimal u corresponding to (16)
% If independently used, uncomment N_sample and sampling part.


figure;
hold on


Xu=cell(1,Nu);

% randomly picking a BRS index
% N=numNodes;%randi([1,numNodes]);

for N = 1:numNodes% numNodes %indices %last step  %flip(1:numNodes)
    
    disp(N)
    % plotting two consecative BRSs
    % Initial Set of Lambda1 colored in green
    plot(T{N},[1 2],'EdgeColor',[0.3,0.3,0.3],'FaceColor','g','FaceAlpha',0.05,'linewidth',2)

    % target set or Lamba2 colored in blue 
    % plot(T{N+1},[1 2],'EdgeColor',[0.3,0.3,0.3],'FaceColor','b','FaceAlpha',0.05,'linewidth',2)
    
    % scatter(states_gaus(1,:), states_gaus(2,:), 'b.')
    % scatter(states_rad(1,:), states_rad(2,:), 'b.')
    % scatter(states_ex(1,:), states_ex(2,:), 'b.')
    % scatter(states_uni(1,:), states_uni(2,:), 'b.')
    % scatter(states_bd(1,:), states_bd(2,:), 'b.')


    % Driving randomly generated values of x in the BRS Lambda1 to the BRS Lambda2 using (16)
    % N_Sample=1000;

    
    % % collect state x1 for the current time step and the optimal input u
    % % for controller training
    % current_state = zeros(3, N_Sample);
    % optimal_control = zeros(2, N_Sample);
     
    
    % for j=1:N_Sample
    %     x1 = states_extreme(:,j); % randPoint(T{N},1);
    %     % control value using (16)
    %     % u_opt=Dubin_Car_Control(x1,Trajectory_input(:,N),Rum(:,N),Tp{N});
    %     % control value using (16) for the quadratic version 1
    %     u_opt=Dubin_Car_Control_Quadratic(x1,Trajectory_input(:,N),Rum(:,N),Tp{N});
    %     x2=F_Dubin_Car(x1,u_opt);
    %     X=[x1(1);x2(1)];
    %     Y=[x1(2);x2(2)];
    %     % plotting the trajectories from Lambda1 to Lambda2
    %     plot(X,Y,'k','linewidth',1);
    % 
    % 
    %     current_state(:,j) = x1;
    %     optimal_control(:,j) = u_opt;
    % 
    %
    % end
    
    xlabel('$x_{1}$','interpreter','latex')
    ylabel('$x_{2}$','interpreter','latex')
    set(gca,'fontsize',15)
    set(gca,'ticklabelinterpreter','latex')

    box on

    % warning('Number of nodes is %d',numNodes)
    

    %save(data_path+"states_optu_sampled_z"+num2str(N)+".mat", 'current_state', 'optimal_control');

end

%figure;
%scatter(optimal_control(1,:), optimal_control(2,:), 'filled', 'b')



function samples = uniform_sample_zonotope(c, G, num_samples)
    % Uniformly sample points within a zonotope defined by a center and generators.
    %
    % Parameters:
    %   c          : Center of the zonotope (column vector of size [d, 1])
    %   G          : Generator matrix of the zonotope (size [d, g])
    %   num_samples: Number of points to sample within the zonotope
    %
    % Returns:
    %   samples    : Uniformly sampled points within the zonotope (size [d, num_samples])

    % Get dimensions
    [d, g] = size(G);  % d = dimension, g = number of generators

    % Initialize sample storage
    samples = zeros(d, num_samples);

    % Generate uniform coefficients in the range [-1, 1] for each generator
    for i = 1:num_samples
        % Generate random coefficients for each generator vector uniformly in [-1, 1]
        random_coeffs = -1 + 2 * rand(g, 1);

        % Calculate the sample point as a linear combination of generators + center
        samples(:, i) = c + G * random_coeffs;
    end
end



