clc
clear all
close all

% random seed: 55 for rl, 66 for lu, 77 for ll
sd = 77;

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
N_Sample_ex = 100;
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



