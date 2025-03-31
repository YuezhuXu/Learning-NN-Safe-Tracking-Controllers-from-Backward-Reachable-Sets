clear all
clc
close all

data_path = "optbased_ubounded/";

% random seed: 55 for rl, 66 for lu, 77 for ll
sd = 77;

if sd == 55
    name_str = 'rl';
elseif sd == 66
    name_str = 'lu';
elseif sd == 77
    name_str = 'll';
end


rng(777)

load(data_path+"Dubin_Car_Data_For_Plotting_"+num2str(sd)+"_"+name_str)
load(data_path+"Trajectory_Dubin_Car_"+num2str(sd)+"_"+name_str)


starting_step = 1;


%% 
if ~exist('figures', 'dir')
    mkdir('figures');
end

fig = figure;
hold on
Xt=zonotope([0.5*(Xtl+Xtu),diag(0.5*(Xtu-Xtl))]);
plot(Xt,[1 2],'g','linewidth',2)
Xu=cell(1,Nu);

for i=1:Nu
    Xu{i}=zonotope([CuArray(:,i),diag(0.5*(XuuArray(:,i)-XulArray(:,i)))]);
    plot(Xu{i},[1 2],'k','linewidth',2)
end


Xt=zonotope(Ct,Gt);
plot(Xt,[1 2],'EdgeColor','none','FaceColor','g')

Xu=cell(1,Nu);

for i=1:Nu
Xu{i}=zonotope([CuArray(:,i),GuArray(:,:,i)]);
plot(Xu{i},[1 2],'EdgeColor','none','FaceColor','k')
end

Xs=zonotope([0.5*(Xsl+Xsu),diag(0.5*(Xsu-Xsl))]);
plot(Xs,[1 2],'k','linewidth',2)


for ind=1:numNodes+1  %numNodes+1:-1:1
plot(T{ind},[1 2],'EdgeColor',[0.3,0.3,0.3],'FaceColor',[0.5,0.5,0.5],'FaceAlpha',0.05,'linewidth',1)

hold on
end

% Texts
text(mean(Xtl(1))+0.75, mean(Xtl(2))+0.25, 'Target', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontSize', 12, 'Interpreter', 'latex');
for i=1:Nu
text(CuArray(1, i), CuArray(2, i), sprintf('O%d', i), 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', 'FontSize', 12, 'Interpreter', 'latex', 'Color', 'w');
end
c1 = center(T{1});
G1 = T{1}.generators; % Generator matrix of the zonotope


% Change the text position if needed
% 55, 66: c1(2)+0.1,77: c1(2)-0.5
if sd == 77
    text_pos = -0.5;
else
    text_pos = 0.1;

end

text(c1(1)+0.5, c1(2)+text_pos, 'Start', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 12, 'Interpreter', 'latex');

xlabel('$x_{1}$','interpreter','latex')
ylabel('$x_{2}$','interpreter','latex')
set(gca,'fontsize',15)
set(gca,'ticklabelinterpreter','latex')
xlim([Xsl(1),Xsu(1)])
ylim([Xsl(2),Xsu(2)])
grid on
box on


%%
% current_model_epoch = 0;

% disturbance radius
er=[3e-3;3e-3;1.5e-2];

numNodes

% In terms of time step, Tp{N} <--> T{N+1}
% We want any sampling points in T{N} to be driven inside Tp{N} in the next step.
% Because matlab indices start from 1, all input indices are added by 1.
% At time step 0: sample in T{1}, drive towards Tp{1}. 
% Nominal input is trajectory_input(:,1) and the trained neural controller
% is model_params_step_1_randxxx_xx.mat

% We first generate test sets: uniform, outside, benchmark
N_Sample_test_uni = 100;

% Sampling uniformly (regular)
initial_states_uni = randPoint(T{starting_step}, N_Sample_test_uni, "uniform");%+[0.02;0.02;0.02];

%% 
% From outside the initial zonotope
N_Sample_test_out = 1000;


% Directly sample from enlarged zonotope uniformly.
c1 = center(T{starting_step});
G1 = T{starting_step}.generators;


% We try different level of points being pushed outside the zonotope
tau = 0.5; % Small perturbation to push points outside
if tau == 0.1
    clr = 'red';
elseif tau == 0.2
    clr = 'cyan';
elseif tau == 0.3
    clr = 'blue';
elseif tau == 0.5
    clr = '#A52A2A';
end
    
Z_enlarged = zonotope(c1, (1 + tau)*G1);
initial_states_out = randPoint(Z_enlarged, N_Sample_test_out, "uniform");


%%
% Benchmark starting point
initial_states_ben = [1;0.8;0];


%%
% Run simulations
% Choose from uni, out, ben
sim_case = 'uni';

if strcmp(sim_case, 'uni')
    N_Sample_test = N_Sample_test_uni;
    initial_states = initial_states_uni;
    clr = 'b';
elseif strcmp(sim_case,'out')
    % check if starting from lu
    if ~strcmp(name_str,'lu')
        error('Wrong case! Check the starting point.');
    end
    N_Sample_test = N_Sample_test_out;
    initial_states = initial_states_out;
elseif strcmp(sim_case, 'ben')
    if  ~strcmp(name_str,'ll')
        error('Wrong case! Check the starting point.');
    end
    N_Sample_test = 1;
    initial_states = initial_states_ben;
    clr = 'r';
end


tic;
count_hitob = 0;
success = 0;
count_inpout = 0;

%%
us_NN = [];
zs_NN = [];
for i = 1:N_Sample_test
    i
    flag = 0;
    % Record the state in all time steps
    traj = zeros(size(initial_states,1),numNodes);
    state_curr = initial_states(:,i); 
    % plot(state_curr(1), state_curr(2),'o', 'MarkerSize', 2, 'MarkerEdgeColor', clr, 'MarkerFaceColor', clr);
     
    traj(:,1) = state_curr; 
    

    for N = starting_step:numNodes-1
        params_NN = load("optbased_ubounded\\trained_controller_each_step\\model_params_step"+num2str(N)+"_rand"+num2str(sd)+"_"+ name_str+".mat");
        % disp(params_NN.Ru)
        c1 = center(T{N});
        cu = Trajectory_input(:, N);
        Ru = Rum(:, N);

        z_NN = middle_layers(c1,cu,state_curr, params_NN);
        u_NN = input_NN(c1, cu, Ru, state_curr, params_NN);
       
        % Clipping
        u_NN(1) = max(min(u_NN(1), 0.4), -0.4);
        u_NN(2) = max(min(u_NN(2), 0.25), -0.25);

        if sum(u_NN<[-0.4; -0.25])+sum(u_NN>[0.4; 0.25])>0
            disp("Sample")
            i
            disp("Time step")
            N        
            disp("Input")
            u_NN
        end

        zs_NN = [zs_NN z_NN];
        us_NN = [us_NN u_NN];
        
        % Dubin_Car_Control_Quadratic(state_curr,cu,Ru,Tp{N});
        % add disturbances in of N(0, er)
        disturbance = (2 * er .* rand(3, 1)) - er;
        state_next = F_Dubin_Car(state_curr,u_NN)+disturbance;

        % For checking success. Check if it hits any obstacle
        if (contains(Xu{1}, state_next) || contains(Xu{2}, state_next)  || contains(Xu{3}, state_next))
            count_hitob = count_hitob + 1;
            flag = 1;
            disp('Hit obstable.')
        end

        % Check if it is outside the bound
        if ~(contains(Xs, state_next))
            flag = 1;
            disp('Outside the bound')
        end


        % For plotting
        X_NN=[state_curr(1);state_next(1)];
        Y_NN=[state_curr(2);state_next(2)];


        linewidth = 1;
        markersize = 2;
        linestyle = '-';

       
        plot(X_NN, Y_NN, linestyle, 'Color', clr, 'LineWidth', linewidth);
        

        state_curr = state_next;
        
        plot(state_curr(1), state_curr(2),'o', 'MarkerSize', 1.5, 'MarkerEdgeColor', clr, 'MarkerFaceColor', clr);
        
        
        
        traj(:,N+1) = state_curr;

    end


    % For checking success
    if flag == 0 && contains(Xt,state_curr)
        success = success+1;
    end
    
end
time_used = toc

success_rate = success/N_Sample_test

if strcmp(sim_case,'out')
    fig_name = ['trajectory_plot_' sim_case '_' num2str(10*tau) '_' name_str '.eps'];
else 
    fig_name = ['trajectory_plot_' sim_case '_' name_str '.eps'];
end


%%
% Step 1: Create inset axes at bottom-left
if strcmp(sim_case,'out')
    inset_axes = axes('Position', [0.65 0.2 0.2 0.2]);
    hold(inset_axes, 'on');
    box(inset_axes, 'on');

    % Step 2: Plot zonotope INTO the inset using 'Parent'
    plot(T{starting_step}, [1 2], ...
        'EdgeColor', 'k', ...
        'FaceColor', [0.8 0.8 0.8], ...
        'FaceAlpha', 0.5, ...
        'Parent', inset_axes);  % <- ensures zonotope plots inside inset

    % Step 3: Plot initial sampled states INTO the inset explicitly
    plot(inset_axes, initial_states(1,:), initial_states(2,:), ...
         '.', 'Color', clr, 'MarkerSize', 5);

    % Step 4: Zoom range — check this matches your data!
    c = center(T{starting_step});
    xlim(inset_axes, [c(1)-0.3, c(1)+0.3]);
    ylim(inset_axes, [c(2)-0.3, c(2)+0.3]);
    % Step 5: Remove ticks/labels
    set(inset_axes, 'XTick', [], 'YTick', [], ...
                    'XColor', 'none', 'YColor', 'none');
    % Step 6: Bring to front (optional)
    uistack(inset_axes, 'top');


    h_rect = rectangle('Position', [c(1)-0.4, c(2)-0.4, 0.8, 0.8], ...
              'EdgeColor', 'k', 'LineStyle', '-', 'LineWidth', 1.2);

    xlim([c(1)-0.4, c(1)+0.4]);
    ylim([c(2)-0.4, c(2)+0.4]);
end

print(fig, fullfile('optbased_ubounded/figures', fig_name), '-depsc', '-opengl');


%%
function u = input_NN(c1, cu, Ru, x, params_NN)
    % Normalize the input with respect to c1
    z = x - c1;
    
    % Layer 1: fc1
    z = relu(params_NN.fc1_weight * z + params_NN.fc1_bias');
    
    % Layer 2: fc2
    z = relu(params_NN.fc2_weight * z + params_NN.fc2_bias');
    
    % Layer 3: fc3
    z = relu(params_NN.fc3_weight * z + params_NN.fc3_bias');
    
    % Layer 4: fc4
    z = relu(params_NN.fc4_weight * z + params_NN.fc4_bias');
    
    % Link-back layer (fclinkback) with element-wise multiplication
    z = (x-c1) .* (params_NN.fclinkback_weight * z + params_NN.fclinkback_bias');
    
    % Final output layer (fcout) with ReLU activation, then tanh
    % Should be no bias and we remove the relu
    % z = relu(params_NN.fcout_weight * z + params_NN.fcout_bias');

    z = params_NN.fcout_weight * z; %relu(params_NN.fcout_weight * z);
    z = tanh(z); % [-1,1]
    
    % disp(params_NN.Rusc)
    % disp(Ru)
    Ru_trained = params_NN.Ru'; %2*1
    % Ru_trained = params_NN.Rusc' .* Ru;
    % Scale the output to the range [cu - Ru, cu + Ru]
    % u = (cu - Ru_trained) + 2 * Ru_trained .* z;
    u = cu + Ru_trained .* z;
    

        
    % % Hard constraint on u to be within [cu - Ru, cu + Ru]
    % lower_bound = cu - Ru;
    % upper_bound = cu + Ru;
    % u = min(max(u, lower_bound), upper_bound);  % Clamp u within bounds
    
end

function z = middle_layers(c1, cu, x, params_NN)
    % Normalize the input with respect to c1
    z = x - c1;
    
    % Layer 1: fc1
    z = relu(params_NN.fc1_weight * z + params_NN.fc1_bias');
    
    % Layer 2: fc2
    z = relu(params_NN.fc2_weight * z + params_NN.fc2_bias');
    
    % Layer 3: fc3
    z = relu(params_NN.fc3_weight * z + params_NN.fc3_bias');
    
    % Layer 4: fc4
    z = relu(params_NN.fc4_weight * z + params_NN.fc4_bias');  
    
    z = params_NN.fclinkback_weight * z + params_NN.fclinkback_bias';

    % Link-back layer (fclinkback) with element-wise multiplication
    z = (x-c1) .* z;
    
    z = params_NN.fcout_weight * z; %relu(params_NN.fcout_weight * z);
    
    % z = tanh(z); % [-1,1]
    
end





% Helper function: ReLU activation
function y = relu(x)
    y = max(0, x);
end

% Helper function: Tanh activation
function y = tanh(x)
    % Calculate the hyperbolic tangent of each element in x
    y = (exp(x) - exp(-x)) ./ (exp(x) + exp(-x));
end

