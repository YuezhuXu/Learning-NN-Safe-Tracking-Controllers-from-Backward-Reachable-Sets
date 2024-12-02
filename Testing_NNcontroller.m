clear
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


rng(456)

load(data_path+"Dubin_Car_Data_For_Plotting_"+num2str(sd)+"_"+name_str)
load(data_path+"Trajectory_Dubin_Car_"+num2str(sd)+"_"+name_str)

figure
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
%plot(T{ind},[1 2],'EdgeColor','none','FaceColor',[0.5,0.5,0.5],'FaceAlpha',0.2)

hold on
end
%plot(T{ind},[1 2],'EdgeColor',[0.3,0.3,0.3],'FaceColor',[0.5,0.5,0.5],'FaceAlpha',0.5)

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
text(c1(1)+0.5, c1(2)-0.35, 'Start', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 12, 'Interpreter', 'latex');

xlabel('$x_{1}$','interpreter','latex')
ylabel('$x_{2}$','interpreter','latex')
set(gca,'fontsize',15)
set(gca,'ticklabelinterpreter','latex')
xlim([Xsl(1),Xsu(1)])
ylim([Xsl(2),Xsu(2)])
grid on
box on





numNodes


% disturbance radius
er=[1e-3;1e-3;5e-3];
starting_step = 1;

% In terms of time step, Tp{N} <--> T{N+1}
% We want any sampling points in T{N} to be driven inside Tp{N} in the next step.
% Because matlab indices start from 1, all input indices are added by 1.
% At time step 0: sample in Tp{1}, drive towards T{1}. 
% Nominal input is trajectory_input(:,1) and the trained neural controller
% is model_params_step_1_randxxx_xx.mat

% We first generate test sets: uniform, outside, benchmark
N_Sample_test_uni = 100;

% Sampling uniformly
initial_states_uni = randPoint(T{starting_step}, N_Sample_test_uni, "uniform");%+[0.02;0.02;0.02];
% Benchmark starting point
initial_states_ben = [1;0.8;0];

N_Sample_test_out = 100;
epsilon = 0.03; % Small perturbation to push points outside
c_proj = c1(1:2);              % Project the center onto dimensions 1 and 2
G_proj = G1(1:2, :);            % Project the generators onto dimensions 1 and 2
Z_proj = zonotope(c_proj, G_proj); % Create a 2D zonotope object
boundary_points = polygon(Z_proj); 

initial_states_out = zeros(size(G1, 1), N_Sample_test_out);

% Solve for each boundary point
for i = 1:N_Sample_test_out
    % Boundary point in 2D
    b_proj = boundary_points(:, i) - c_proj;
    
    % Optimization setup for r
    H = eye(size(G_proj, 2));  % Quadratic term (minimizing ||r||_2)
    f = zeros(size(G_proj, 2), 1);  % Linear term
    Aeq = G_proj;               % Equality constraint (projected generators)
    beq = b_proj;               % Target projected point
    lb = -ones(size(G_proj, 2), 1); % Lower bounds for r
    ub = ones(size(G_proj, 2), 1);  % Upper bounds for r
    
    % Solve using quadprog
    r = quadprog(H, f, [], [], Aeq, beq, lb, ub);
    % Reconstruct the original high-dimensional point
    initial_states_out(:, i) = c1 + (1+epsilon)*G1 * r;
    
end

%%
% Run simulations
% Choose from uni, out, ben
% Reuse the module if two cases are ploted on one graph
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
    clr = 'r';
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



for i = 1:N_Sample_test
    i
    flag = 0;
    % Record the state in all time steps
    traj = zeros(size(initial_states,1),numNodes);
    state_curr = initial_states(:,i); 
    plot(state_curr(1), state_curr(2),'o', 'MarkerSize', 2, 'MarkerEdgeColor', clr, 'MarkerFaceColor', clr);
     
    traj(:,1) = state_curr; 

    for N = starting_step:numNodes
        params_NN = load("optbased_ubounded\\trained_controller_rcac\\model_params_step"+num2str(N)+"_rand"+num2str(sd)+"_"+ name_str+".mat");
        c1 = center(T{N});
        cu = Trajectory_input(:, N);
        Ru = Rum(:, N);
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


        % Dubin_Car_Control_Quadratic(state_curr,cu,Ru,Tp{N});
        % add disturbances in [-er, er]
        disturbance = (2 * er .* rand(3, 1)) - er;
        state_next = F_Dubin_Car(state_curr,u_NN)+disturbance;
        % Check if it hits any obstacle
        if (contains(Xu{1}, state_next) || contains(Xu{2}, state_next)  || contains(Xu{3}, state_next))
            count_hitob = count_hitob + 1;
            flag = 1;
        end
        
        % Check if it is outside the bound
        if ~(contains(Xs, state_next))
            flag = 1;
        end
        

        % For plotting
        X_NN=[state_curr(1);state_next(1)];
        Y_NN=[state_curr(2);state_next(2)];
        % Use blue for starting points within Lambda0,
        % red for starting
        % points outside Lambda0
        plot(X_NN,Y_NN,clr+"-",'linewidth',1);
        

        state_curr = state_next;

        % Use blue for starting points within Lambda0,
        % red for starting points outside Lambda0
        plot(state_curr(1), state_curr(2),'o', 'MarkerSize', 2, 'MarkerEdgeColor', clr, 'MarkerFaceColor', clr);
        traj(:,N+1) = state_curr;

    end

    if flag == 0 && contains(Xt,state_curr)
        success = success+1;
    end
end
time_used = toc

success_rate = success/N_Sample_test




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
    
    % Final output layer (fcout) with ReLU activation, then sigmoid
    z = relu(params_NN.fcout_weight * z + params_NN.fcout_bias');
    % z = sigmoid(z);  % Apply sigmoid to bring z within [0, 1]
    z = tanh(z);

    Ru_trained = params_NN.Ru';
    % Scale the output to the range [cu - Ru, cu + Ru]
    % u = (cu - Ru_trained) + 2 * Ru_trained .* z;
    u = cu + Ru_trained .* z;
    

        
    % % Hard constraint on u to be within [cu - Ru, cu + Ru]
    % lower_bound = cu - Ru;
    % upper_bound = cu + Ru;
    % u = min(max(u, lower_bound), upper_bound);  % Clamp u within bounds
    
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

