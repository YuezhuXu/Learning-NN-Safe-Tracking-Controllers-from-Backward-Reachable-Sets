clc
clear all
close all

% random seed: 333 for rl, 222 for lu, 111 for ll
sd = 222;

if sd == 333
   name_str = 'rl';
elseif sd == 222
   name_str = 'lu';
elseif sd == 111
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
    

    c2 = center(Tp{N});
    states_ex = randPoint(T{N}, N_Sample_ex, "extreme");


    G2 = Tp{N}.generators;
    
    save(data_path+"\\sampling\\states_sampled_extreme_z"+num2str(N)+"_rand"+num2str(sd)+"_"+name_str+".mat", 'cu', 'Ru', 'states_ex', 'c1', 'c2', 'G2');
    save(data_path+"\\sampling\\states_sampled_uniform_z"+num2str(N)+"_rand"+num2str(sd)+"_"+name_str+".mat", 'cu', 'Ru', 'states_uni', 'c1', 'c2', 'G2');
    
end



%%
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



