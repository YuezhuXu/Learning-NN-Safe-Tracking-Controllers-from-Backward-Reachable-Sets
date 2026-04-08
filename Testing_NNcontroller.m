clear all
clc
close all
warning('off','all') 



% random seed: 333 for rl, 222 for lu, 111 for ll
sd = 111;

if sd == 333
   name_str = 'rl';
elseif sd == 222
   name_str = 'lu';
elseif sd == 111
   name_str = 'll';
end


rng(123)

sim_case = 'uni'; % 'uni', 'out' (need to set vareps), 'ben_BRSopt', 'ben_noBRS', 'ben_CBF'
vareps = 0.3; % Small perturbation to push points outside, for 'out', 0.1, 0.2, 0.3


% check the case and specify data path
if strcmp(sim_case, 'uni')
    data_path = 'learning_BRS_instructed\\';
elseif strcmp(sim_case,'out')
    % check if starting from lu
    if ~strcmp(name_str,'lu')
        error('Wrong case! Check the starting point.');
    end
    data_path = 'learning_BRS_instructed\\';
elseif strcmp(sim_case, 'ben_BRSopt')
    if  ~strcmp(name_str,'ll')
        error('Wrong case! Check the starting point.');
    end
    data_path = 'learning_BRS_instructed\\';
    clr = 'r';
elseif strcmp(sim_case, 'ben_noBRS')
    if  ~strcmp(name_str,'ll')
        error('Wrong case! Check the starting point.');
    end
    data_path = 'learning_without_BRS\\';
    clr = 'r';
elseif strcmp(sim_case, 'ben_CBF')
    if  ~strcmp(name_str,'ll')
        error('Wrong case! Check the starting point.');
    end
    data_path = 'solving_with_CBF\\';
    addpath(data_path)
end



load(data_path+"Dubin_Car_Data_For_Plotting_"+num2str(sd)+"_"+name_str)
load(data_path+"Trajectory_Dubin_Car_"+num2str(sd)+"_"+name_str)


starting_step = 1;
end_step = numNodes-1;
figure;
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



% Plotting the safe boxes
for ind=starting_step:end_step


    % safe_rec=zonotope(X(:,ind),diag(Rx(:,ind)));
    % plot(safe_rec,[1 2],'EdgeColor','none','FaceColor',[0.3010 0.7450 0.9330],'FaceAlpha',0.1)

    x_center = X(1, ind);
    y_center = X(2, ind);

    % Extract symmetric radius (half-width)
    % leave a tiny margin 0.01
    rx = Rx(1, ind)-0.01; 
    ry = Rx(2, ind)-0.01;

    % % Symmetric hyper-rectangle
    % % Define corners (Center +/- Radius)
    % x_min = x_center - rx;
    % x_max = x_center + rx;
    % y_min = y_center - ry;
    % y_max = y_center + ry;

    % Asymmetric hyper-rectangle
    rx_minus = Rx_minus(1, ind);
    rx_plus = Rx_plus(1, ind);
    ry_minus = Rx_minus(2, ind);
    ry_plus = Rx_plus(2, ind);

    % Define corners of the box
    x_min = x_center - rx_minus;
    x_max = x_center + rx_plus;
    y_min = y_center - ry_minus;
    y_max = y_center + ry_plus;

    % Rectangle corners (clockwise)
    x_box = [x_min, x_max, x_max x_min];
    y_box = [y_min, y_min, y_max, y_max];


    % Check containment in operating region (use Xsl/Xsu directly)
    if x_min < Xsl(1) || x_max > Xsu(1) || ...
       y_min < Xsl(2) || y_max > Xsu(2)
        fprintf("Step %d: Safe box exceeds operating region bounds!\n", ind);
        violation_flags(ind) = true;
    end

    % Check intersection with any obstacle (Xu{i})
    for i = 1:length(Xu)
        Xu_i = Xu{i};
        Xu_c = Xu_i.Z(:,1);
        Xu_g = abs(Xu_i.Z(:,2:end));
        Xu_bounds_x = [Xu_c(1) - sum(Xu_g(1,:)), Xu_c(1) + sum(Xu_g(1,:))];
        Xu_bounds_y = [Xu_c(2) - sum(Xu_g(2,:)), Xu_c(2) + sum(Xu_g(2,:))];

        % Axis-aligned bounding box intersection in 2D
        intersects = (x_min <= Xu_bounds_x(2)) && (x_max >= Xu_bounds_x(1)) && ...
                     (y_min <= Xu_bounds_y(2)) && (y_max >= Xu_bounds_y(1));

        if intersects
            fprintf("Step %d: Safe box intersects with obstacle %d!\n", ind, i);
            violation_flags(ind) = true;
            break;  % no need to check further obstacles
        end
    end

    % % If all good, plot them
    % Fill interior 
    fill(x_box, y_box,[0.3010 0.7450 0.9330], ...
         'FaceAlpha', 0.05, 'EdgeColor', [0.2 0.6 0.9], 'LineWidth', 1.5);

end


% % Plot nominal trajectory: Solid line (-), Black (k), Circle marker (o) filled black 
% plot(X(1, starting_step:end_step), X(2, starting_step:end_step), ...
%     '-ko', ...                    
%     'LineWidth', 1.5, ...          
%     'MarkerSize', 2, ...           
%     'MarkerFaceColor', 'k');


for ind=starting_step:end_step
    plot(T{ind},[1 2],'EdgeColor',[0.3,0.3,0.3],'FaceColor',[0.5,0.5,0.5],'FaceAlpha',0.05,'linewidth',1)
    plot(T{ind},[1 2],'EdgeColor','none','FaceColor',[0.5,0.5,0.5],'FaceAlpha',0.2)
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
% rl, lu: c1(2)+0.1, ll: c1(2)-0.5
if sd == 111
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

%% Preparation for different cases
% disturbance radius
er=[3e-3;3e-3;1.5e-2];


% In terms of time step, Tp{N} <--> T{N+1}
% We want any sampling points in T{N} to be driven inside Tp{N} in the next step.
% Because matlab indices start from 1, all input indices are added by 1.
% At time step 0: sample in T{1}, drive towards Tp{1}. 
% Nominal input is trajectory_input(:,1) and the trained neural controller
% is model_params_step_1_randxxx_xx.mat



switch sim_case
    case 'uni'
        N_Sample_test = 1000;
        initial_states = randPoint(T{starting_step}, N_Sample_test, "uniform");
        clr = 'b';
        
    case 'out'
        N_Sample_test = 1000;
        
        c1 = center(T{starting_step});
        G1 = T{starting_step}.generators;
        
        switch vareps
            case 0.1
                clr = 'red';
            case 0.2
                clr = 'cyan';
            case 0.3
                clr = '#A52A2A';
        end
            
        Z_enlarged = zonotope(c1, (1 + vareps)*G1);
        initial_states = randPoint(Z_enlarged, N_Sample_test, "uniform");
        
    case 'ben_BRSopt'
        N_Sample_test = 1;
        initial_states = [1; 0.8; 0];
        clr = 'r';
        
    case 'ben_noBRS'
        N_Sample_test = 1000;
        initial_states = randPoint(T{starting_step}, N_Sample_test, "uniform");
        
    case 'ben_CBF'
        N_Sample_test = 1000;
        initial_states = randPoint(T{starting_step}, N_Sample_test, "uniform");
end







%% Run simulations to test (for uni, out, ben_BRSopt, ben_noBRS)
% If run for ben_CBF, go to line 499
% Run and record time
us_NN = [];

violation_score_all = inf(N_Sample_test,1);


delete(gcp('nocreate'))
parpool;
tic;
parfor i = 1:N_Sample_test   
    i
    traj = zeros(size(initial_states,1),numNodes);
    state_curr = initial_states(:,i); 

    traj(:,1) = state_curr; 


    violation_score_traj = 0;
    for N = starting_step:end_step 

        % Testing NN controllers trained using our methods
        params_NN = load(data_path+"trained_controller_each_step\\model_params_step"+num2str(N)+"_rand"+num2str(sd)+"_"+ name_str+".mat");


        c1 = center(T{N});
        cu = Trajectory_input(:, N);
        Ru = Rum(:, N);

        % For conformal prediction
        % Safe hyper-rectangle
        hyperrec_c = X(:, N);      % 3×1 vector
        hyperrec_minus = Rx_minus(:, N); % 3×1 vector
        hyperrec_plus = Rx_plus(:, N); % 3×1 vector


        % Hyper-rectangle bounds
        lower_corner = hyperrec_c - hyperrec_minus;
        upper_corner = hyperrec_c + hyperrec_plus;

        if any(state_curr < lower_corner) || any(state_curr > upper_corner)
            disp("Out of the safe hyper-rectangle");
            state_curr
            lower_corner
            upper_corner

            % This step violates the safety hyper-rectangle
            violation_score =  distance_to_box(state_curr, lower_corner, upper_corner);
            disp("Violation at stage")
            N
            violation_score_traj = violation_score_traj + violation_score;
        else
            violation_score = 0;
        end


        % Moving forward
        u_NN = input_NN(c1, cu, Ru, state_curr, params_NN);


        % Clipping
        u_NN(1) = max(min(u_NN(1), 0.4), -0.4);
        u_NN(2) = max(min(u_NN(2), 0.25), -0.25);


        us_NN = [us_NN u_NN];

        % add disturbances in of (0, er)
        disturbance = (2 * er .* rand(3, 1)) - er;
        state_next = F_Dubin_Car(state_curr,u_NN)+disturbance;


        state_curr = state_next;

        traj(:,N+1) = state_curr;

    end


    violation_score_all(i) = violation_score_traj;



end
time_used = toc;
delete(gcp('nocreate'))

disp("Checking violation")
sum(violation_score_all)
simu_time_each = time_used/N_Sample_test




%% Run and plot (100 for demonstration)  (for uni, out, ben_BRSopt, ben_noBRS)
us_NN = [];


% 100 for uni, out, ben_noBRS; 1 for ben_BRSopt
if strcmp(sim_case, 'ben_BRSopt')
    N_plot = 1;
else
    N_plot = 100;
end


for i = 1:N_plot
    i
    traj = zeros(size(initial_states,1),numNodes);
    state_curr = initial_states(:,i); 
    plot(state_curr(1), state_curr(2),'o', 'MarkerSize', 2, 'MarkerEdgeColor', clr, 'MarkerFaceColor', clr);

    traj(:,1) = state_curr; 

    for N = starting_step:end_step

        % Testing NN controllers trained using our methods
        params_NN = load(data_path+"trained_controller_each_step\\model_params_step"+num2str(N)+"_rand"+num2str(sd)+"_"+ name_str+".mat");

        % disp(params_NN.Ru)
        c1 = center(T{N});
        cu = Trajectory_input(:, N);
        Ru = Rum(:, N);



        % Check if test state is inside the safe set
        % Safe hyper-rectangle
        hyperrec_c = X(:, N);      % 3×1 vector
        hyperrec_minus = Rx_minus(:, N); % 3×1 vector
        hyperrec_plus = Rx_plus(:, N); % 3×1 vector


        % Hyper-rectangle bounds
        lower_corner = hyperrec_c - hyperrec_minus;
        upper_corner = hyperrec_c + hyperrec_plus;     


        % Moving forward
        u_NN = input_NN(c1, cu, Ru, state_curr, params_NN);


        % Clipping
        u_NN(1) = max(min(u_NN(1), 0.4), -0.4);
        u_NN(2) = max(min(u_NN(2), 0.25), -0.25);


        us_NN = [us_NN u_NN];

        % Dubin_Car_Control_Quadratic(state_curr,cu,Ru,Tp{N});
        % add disturbances in of U(0, er)
        disturbance = (2 * er .* rand(3, 1)) - er;
        state_next = F_Dubin_Car(state_curr,u_NN)+disturbance;


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


end










%% Adding zoom in plots for the case out
% if strcmp(sim_case,'out')
%     inset_axes = axes('Position', [0.65 0.2 0.2 0.2]);
%     hold(inset_axes, 'on');
%     box(inset_axes, 'on');
% 
% 
%     plot(T{starting_step}, [1 2], ...
%         'EdgeColor', 'k', ...
%         'FaceColor', [0.8 0.8 0.8], ...
%         'FaceAlpha', 0.5, ...
%         'Parent', inset_axes);  % <- ensures zonotope plots inside inset
% 
% 
%     plot(inset_axes, initial_states(1,:), initial_states(2,:), ...
%          '.', 'Color', clr, 'MarkerSize', 5);
% 
% 
%     c = center(T{starting_step});
%     xlim(inset_axes, [c(1)-0.3, c(1)+0.3]);
%     ylim(inset_axes, [c(2)-0.3, c(2)+0.3]);
% 
%     set(inset_axes, 'XTick', [], 'YTick', [], ...
%                     'XColor', 'none', 'YColor', 'none');
% 
%     uistack(inset_axes, 'top');
% 
% 
%     h_rect = rectangle('Position', [c(1)-0.4, c(2)-0.4, 0.8, 0.8], ...
%               'EdgeColor', 'k', 'LineStyle', '-', 'LineWidth', 1.2);
% 
%     xlim([c(1)-0.4, c(1)+0.4]);
%     ylim([c(2)-0.4, c(2)+0.4]);
% end


%% Run simulations (for ben_CBF)
gamma_k = 0.4;


tic;
fail = 0;
for i = 1:N_Sample_test
    i
    xk = initial_states(:,i); 
    traj = zeros(size(initial_states,1), numNodes);
    traj(:,1) = xk;

    for k = starting_step:end_step 
        % k
        xref_next = X(:,k+1);
        [uk, exitflag] = solve_cbf_dubins_one_step(xk, xref_next, gamma_k, Xs, Xu, er, Ul, Uu, @F_Dubin_Car);
        if exitflag <= 0
            disp('The optimization problem is infeasible at stage')
            k
            fail = fail + 1;
            break
        end
    
        wk = (2*rand(3,1)-1).*er;
        xnext = F_Dubin_Car(xk, uk) + wk;

    
        xk = xnext;
        traj(:,k+1) = xk;
    end
end
time_used = toc;
average_time_used = time_used/N_Sample_test
success_rate = 1-fail/N_Sample_test

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
    z = params_NN.fcout_weight * z; %relu(params_NN.fcout_weight * z);
    z = tanh(z); % [-1,1]
    

    Ru_trained = params_NN.Ru'; %2*1
    % Scale the output to the range [cu - Ru, cu + Ru]
    u = cu + Ru_trained .* z;
        
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




function d = distance_to_box(x, lower_corner, upper_corner)
    % Computes L-infinity distance from point x to axis-aligned box
    % defined by lower_corner and upper_corner
    %
    % x:           nx1 state
    % lower_corner: nx1 lower bounds of the box
    % upper_corner: nx1 upper bounds of the box

    % Clamp x into the box to get the nearest point y in the box
    y_clamped = min(max(x, lower_corner), upper_corner);

    % Compute L-infinity distance
    d = norm(x - y_clamped, inf);
end







