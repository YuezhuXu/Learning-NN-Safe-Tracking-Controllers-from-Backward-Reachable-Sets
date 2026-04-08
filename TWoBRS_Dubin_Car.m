clc
clear all
close all


sd = 111;

if sd == 333
   name_str = 'rl';
elseif sd == 222
   name_str = 'lu';
elseif sd == 111
   name_str = 'll';
end


rng(123)



for data_path = ["learning_BRS_instructed\\","learning_without_BRS\\"]

    load(data_path+'Dubin_Car_Data_For_Plotting_'+num2str(sd)+"_"+name_str)
    
    
    % Get cu,Ru, sampled states xu
    % In terms of time step, T{N+1} corresponds to Tp{N} (Tp{N} <--> T{N+1})
    % We want any sampling points in T{N} to be driven inside Tp{N} in the next step.
    N_Sample_ex = 60;
    N_Sample_uni = 40;
    
    
    states_starting = randPoint(T{1}, N_Sample_uni, "uniform");
    for N = 1:numNodes
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
        states_bd = randPoint(T{N}, N_Sample_ex, "boundary");
    
    
        G2 = Tp{N}.generators;
        
        save(data_path+"\\sampling\\states_sampled_extreme_z"+num2str(N)+"_rand"+num2str(sd)+"_"+name_str+".mat", 'cu', 'Ru', 'states_ex', 'c1', 'c2', 'G2');
        save(data_path+"\\sampling\\states_sampled_uniform_z"+num2str(N)+"_rand"+num2str(sd)+"_"+name_str+".mat", 'cu', 'Ru', 'states_uni', 'c1', 'c2', 'G2');
        
    end

end



