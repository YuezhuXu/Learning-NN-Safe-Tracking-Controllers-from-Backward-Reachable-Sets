# -*- coding: utf-8 -*-
"""
Created on Fri Oct 18 17:41:09 2024

@author: 22384
"""


# We normalize states x to x-c1 (or x-xc) before it is input in the trainable feedforward part of NN 
# Biases are kept (x-xc)*NN_{before sigmoid}(x-xc)

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
#import matplotlib.pyplot as plt
import numpy as np
import scipy.io
import os
from sklearn.model_selection import train_test_split  # For splitting the data
import time

np.random.seed(42)  


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")



def F_Dubin_Car(x, u):
    
    cos_theta = torch.cos(x[:, 2])  # Apply element-wise cos
    sin_theta = torch.sin(x[:, 2])  # Apply element-wise sin
    
    transformation_matrix = torch.stack([torch.stack([cos_theta, torch.zeros_like(cos_theta)], dim=1),
                                         torch.stack([sin_theta, torch.zeros_like(sin_theta)], dim=1),
                                         torch.stack([torch.zeros_like(cos_theta), torch.ones_like(cos_theta)], dim=1)], dim=1)
    
    y = x + torch.bmm(transformation_matrix, u.unsqueeze(2)).squeeze(2)
    
    return y

def init_he(m):
    if isinstance(m, nn.Linear):
        nn.init.kaiming_uniform_(m.weight, nonlinearity='relu')
        # nn.init.zeros_(m.weight)
        nn.init.zeros_(m.bias)
        
        
def save_model_params(model, filename):
    params = {}
    for name, param in model.named_parameters():
        # Replace periods in parameter names with underscores for MATLAB compatibility
        safe_name = name.replace('.', '_')
        # Convert each parameter tensor to a NumPy array
        param_np = param.detach().cpu().numpy().astype(np.float64)  # Ensure compatible data type
        params[safe_name] = param_np
    if 'Ru' not in params:
        Ru_trained = model.Ru.detach().cpu().numpy().astype(np.float64)
        params['Ru'] = Ru_trained
        
    # Save in MATLAB v5 .mat format
    scipy.io.savemat(filename, params)


    
# Get data from saved .mat file
#rl: 55,90; lu: 66,102; ll: 77,105
sd = 66

if sd == 55:
    ini_str = 'rl'
    numNodes = 90
elif sd == 66:
    ini_str = 'lu'
    numNodes = 102
elif sd == 77:
    ini_str = 'll'
    numNodes = 105
    
    
    

start_all = time.time()
for ind in range(1,numNodes+1):
    start = time.time()

    data_ex = scipy.io.loadmat('sampling/states_sampled_extreme_z'+str(ind)+'_rand'+str(sd)+'_'+ini_str+'.mat')
    #data_gaus = scipy.io.loadmat('states_sampled_gaussian_z'+str(ind)+'_rand'+str(sd)+'.mat')
    #data_rad =  scipy.io.loadmat('states_sampled_radius_z'+str(ind)+'_rand'+str(sd)+'.mat')
    data_uni = scipy.io.loadmat('sampling/states_sampled_uniform_z'+str(ind)+'_rand'+str(sd)+'_'+ini_str+'.mat')
    #data_bd = scipy.io.loadmat('states_sampled_boundary_z'+str(ind)+'_rand'+str(sd)+'.mat')
    
    # Same in all
    cu = torch.tensor(data_uni['cu'], dtype=torch.float32).flatten().to(device)
    # What if makes the Ru trainable ???
    Ru = torch.tensor(data_uni['Ru'], dtype=torch.float32).flatten().to(device)
    c1 = torch.tensor(data_uni['c1'].flatten(), dtype=torch.float32).to(device)
    c2 = torch.tensor(data_uni['c2'].flatten(), dtype=torch.float32).to(device)
    G2 = data_uni['G2']
    G_pseudoinverse = torch.tensor(np.linalg.pinv(G2), dtype=torch.float32).to(device)
    
    
    
    states_ex = data_ex['states_ex'].T  # Shape [x, 3]
    # states_gaus = data_gaus['states_gaus'].T  # Shape [x, 3]
    # states_rad = data_rad['states_rad'].T
    states_uni = data_uni['states_uni'].T
    #states_bd = data_bd['states_bd'].T
    #states_gruni = np.concatenate((states_gaus, states_rad), axis=0)  
    states_uniex = np.concatenate((states_uni, states_ex), axis=0)  
    
    
    # Split data into training (70%) and test (30%) sets from the uniform set
    # Add the extreme sampling to training set
    # states_train_uni, states_test = train_test_split(states_uni, test_size=0.3, random_state=42)
    #states_train = np.concatenate((states_ex, states_train_uni), axis=0)    
    # states_test = states_rad
    states_train, states_val = train_test_split(states_uniex, test_size=0.3)
    
    
    # Define the Neural Network
    class CustomNN(nn.Module):
        def __init__(self, input_size, hidden_size, output_size, c1, cu, initial_Ru):
            super(CustomNN, self).__init__()
            
            # Adding biases for each layer
            self.fc1 = nn.Linear(input_size, hidden_size, bias=True)  
            self.fc2 = nn.Linear(hidden_size, hidden_size, bias=True)
            self.fc3 = nn.Linear(hidden_size, hidden_size, bias=True)
            self.fc4 = nn.Linear(hidden_size, hidden_size, bias=True) # too deep does not help
            self.fclinkback =  nn.Linear(hidden_size, input_size, bias=True)
            self.fcout = nn.Linear(input_size, output_size, bias=True)  
            

            
            # Store cu and Ru for transformation
            self.cu = cu
            # self.Ru = initial_Ru
            # Initialize Ru as a trainable parameter
            self.Ru = nn.Parameter(torch.zeros_like(initial_Ru))#torch.tensor(initial_Ru, dtype=torch.float32))  # initial_Ru is a 2x1 vector
            self.c1 = c1
            
            """
            # Clamping bounds for Ru within the network
            self.Ru_min = torch.tensor([max(self.cu[0] - 0.5, -0.5 - self.cu[0]), max(self.cu[1] - 0.3, -0.3 - self.cu[1])]).to(self.cu.device)
            self.Ru_max = torch.tensor([min(0.5 - self.cu[0], self.cu[0] + 0.5), min(0.3 - self.cu[1], self.cu[1] + 0.3)]).to(self.cu.device)
            """
            

        def forward(self, x):
            # Clamping 
            # self.Ru.data = torch.clamp(self.Ru.data, self.Ru_min, self.Ru_max)
            # Normalize the input before passing through the first layer
            x_normalized = x - self.c1  # Normalize with respect to c1
    
            # Pass through the trainable layers
            x = torch.relu(self.fc1(x_normalized))  # First hidden layer
            x = torch.relu(self.fc2(x))  
            x = torch.relu(self.fc3(x))  
            x = torch.relu(self.fc4(x))  
            x = torch.relu(self.fclinkback(x))  
            
            
            # Multiply the input difference (x - c1) before applying sigmoid to the output
            x = x * x_normalized  # Multiply by (x - c1)
            
            x= torch.relu(self.fcout(x))
            
            '''
            # Sigmoid to scale the output to the range [0, 1]
            x = torch.softmax(x, dim = 1)
            
            # Transform output to the range [cu - Ru, cu + Ru]
            u = (self.cu - self.Ru) + x * (2 * self.Ru)  # Scale and shift to [cu - Ru, cu + Ru]
            '''
            
            # Tanh to scale the output to the range [-1, 1]
            x = torch.tanh(x)
            
            
            # Transform output to the range [cu - Ru, cu + Ru]
            u = self.cu + x * self.Ru  # Scale and shift to [cu - Ru, cu + Ru]
            '''
            # Clamp u to the range [cu - Ru_initial, cu + Ru_initial]
            lower_bound = self.cu - self.Ru
            upper_bound = self.cu + self.Ru
            u = torch.clamp(u, min=lower_bound, max=upper_bound)
            '''
            
            return u
    
    
    # Create the neural network instance
    input_size = 3  # Example input dimension
    hidden_size = 32
    output_size = 2
    
    c1 = torch.tensor(c1, dtype=torch.float32)
    c2 = torch.tensor(c2, dtype=torch.float32)
    net = CustomNN(input_size, hidden_size, output_size, c1, cu, Ru).to(device)
    net.apply(init_he)
    
    # Optimizer
    optimizer = optim.Adam(net.parameters(), lr=1e-3, weight_decay=1e-5)
    

        
    # Define the mean squared error loss
    loss_fn = nn.MSELoss()
    
    # Training loop

    
    num_batches = 1 #4*
    batch_size = len(states_train) // num_batches
    
    # Track losses
    #losses = []
    
    
    '''
    # Initialize adaptive coefficients for each training sample
    adaptive_coefficients = torch.ones(len(states_train), dtype=torch.float32)
    adaptive_coefficients_prev = adaptive_coefficients.clone()  # Track previous coefficients
    '''
    
    #previous_loss_value = float('inf')  # Track the previous loss (start with a high value)
    prev_loss = 1e10
    patience = 5  # Number of epochs to wait for an improvement
    patience_counter = 0  # Counter to keep track of epochs without improvement
    best_val_loss = float('inf')  # Initialize best validation loss with infinity
    for epoch in range(10000):  # Example number of epoc
        
        for i in range(num_batches):
            # Get batch of data based on the current train_indices
            start_idx = i * batch_size
            end_idx = (i + 1) * batch_size
    
            x_batch = torch.tensor(states_train[start_idx:end_idx, :], dtype=torch.float32).to(device)
           
            
            # Forward pass
            u_batch = net(x_batch)
            y_batch = F_Dubin_Car(x_batch, u_batch)
    
            # Expand c2 to match y_batch's shape
            c2_expanded = c2.repeat(y_batch.shape[0], 1)  # Repeat along the batch size
    
            # Compute the residual f(x^s_k, Π(x^s_k)) - c2
            residual = y_batch - c2_expanded
    
            # Apply the pseudoinverse inside the norm
            residual_pinv = torch.matmul(G_pseudoinverse, residual.T).T  # G_pseudoinverse * residual
            # G2 helps
            # residual_pinv = residual
            
            infinity_norms = torch.norm(residual_pinv, p=float('inf'), dim=1)
            
            # Compute the squared norm for each residual, multiplied by the batch-specific coefficients
            # loss = torch.mean(adaptive_coefficients * (torch.norm(residual_pinv, dim=1)**2))
            # loss = torch.mean(adaptive_coefficients * (torch.norm(residual_pinv, dim=1, p = float('inf'))))
             
            # Infinity norm works the best
            # loss = torch.mean(torch.norm(residual_pinv, dim=1, p = float('inf')))
            # loss = torch.mean(torch.exp(torch.norm(residual_pinv, dim=1, p=float('inf')) - 1))
            # loss = torch.mean(torch.exp(torch.relu(infinity_norms - 1)))+torch.norm(net.Ru, p=2)
            loss = torch.mean(torch.exp(torch.relu(infinity_norms - 1))+torch.norm(residual_pinv, dim=1, p = float('inf')))+torch.norm(net.Ru, p=2)
           
            
            '''
            # Consider adding additional penalty on the ones with >1 infinity norms
            # Compute the infinity norm of each vector in `residual_pinv`
            infinity_norms = torch.norm(residual_pinv, p=float('inf'), dim=1)
            
            # Calculate penalty for vectors exceeding the infinity norm constraint
            penalty = torch.mean(torch.exp(torch.relu(infinity_norms - 1)))
            
            # Main loss (e.g., MSE or another primary loss function)
            main_loss = torch.mean(infinity_norms)
            
            
            # Combine main loss with constraint penalty
            constraint_weight = 1  # Adjust this weight as needed
            loss = main_loss + constraint_weight * penalty
            '''
             
            '''
            # For barrier loss function
            # Calculate the infinity norm for each vector in residual_pinv
            inf_norm_residuals = torch.max(torch.abs(residual_pinv), dim=1).values  # Shape: [420]
            
            # Ensure values stay below 1 by clamping, and apply the barrier function
            # Clamping keeps the values slightly below 1 to avoid NaN in the log calculation
            inf_norm_residuals_clamped = torch.clamp(inf_norm_residuals, max=0.99)
            barrier_loss = torch.log(1 - inf_norm_residuals_clamped)  # Barrier term

            # Add a high penalty for any vector whose infinity norm exceeds 1
            penalty = 1000 * torch.relu(inf_norm_residuals - 1)  # Penalty for norm >= 1
            
            # Combine barrier loss and penalty, weighted by adaptive_coefficients
            # loss = torch.mean(adaptive_coefficients * barrier_loss + penalty)
            loss = torch.mean(barrier_loss + penalty)
            '''


 
            # Backpropagation and optimization step
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
        
        
        
        # Check validation loss every 100 epochs
        if epoch % 100 == 0:
            net.eval()
            with torch.no_grad():
                x_val = torch.tensor(states_val, dtype=torch.float32).to(device)
                u_val = net(x_val)
                y_val = F_Dubin_Car(x_val, u_val)
                residual_val = y_val - c2.repeat(y_val.shape[0], 1)
                residual_pinv_val = torch.matmul(G_pseudoinverse, residual_val.T).T
                val_loss_mean = torch.mean(torch.norm(residual_pinv_val, dim=1, p=float('inf')))
                print(f"Validation loss at epoch {epoch}: ", val_loss_mean.item())
            
            # Early stopping check
            if val_loss_mean < best_val_loss:
                best_val_loss = val_loss_mean  # Update best validation loss
                patience_counter = 0  # Reset patience counter
                # Save the best model parameters
                best_params = {name: param.clone().detach() for name, param in net.named_parameters()}
            else:
                patience_counter += 1  # Increment patience counter if no improvement
                
                # Stop training if patience has been exceeded
                if patience_counter >= patience:
                    break
     
            net.train()  # Switch back to training mode after validation
        '''
        if epoch % 50 == 0:
            print(f'Epoch {epoch}, Loss: {loss.item()}')
        '''
    
    
    """
    
    
    for epoch in range(5000):  # Example number of epochs
        for i in range(num_batches):
            # Get batch of data
            x_batch = torch.tensor(states_train[i*batch_size:(i+1)*batch_size,:], dtype=torch.float32)
            u_batch = net(x_batch)
            y_batch = F_Dubin_Car(x_batch, u_batch)
    
            # Expand c2 to match y_batch's shape
            c2_expanded = c2.repeat(y_batch.shape[0], 1)  # Repeat along the batch size
    
            # Compute the residual f(x^s_k, Π(x^s_k)) - c^2
            residual = y_batch - c2_expanded
    
            # Apply the pseudoinverse inside the norm
            residual_pinv = torch.matmul(G_pseudoinverse, residual.T).T  # G_pseudoinverse * residual
    
            # Compute the squared norm for each residual and take the mean
            loss = torch.mean(torch.norm(residual_pinv, dim=1)**2)
    
    
            # Backpropagation and optimization step
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
    
        # Append the loss for tracking
        losses.append(loss.item())
    
        if epoch % 100 == 0:
            print(f'Epoch {epoch}, Loss: {loss.item()}')
    """       
    '''  
    # Optional: Plot losses after training
    plt.plot(losses)
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('Loss over Time')
    plt.show()
    '''
    
    '''
    # Evaluation on the test set
    net.eval()  # Set model to evaluation mode (turns off dropout, etc.)
    
    # Test on test set
    x_test_tensor = torch.tensor(states_test, dtype=torch.float32)
    u_test = net(x_test_tensor)
    y_test = F_Dubin_Car(x_test_tensor, u_test)
    
    # Convert the inputs and outputs to NumPy arrays
    inputs_fromNN_test_np = u_test.detach().cpu().numpy()
    # y_test_np = y_test.detach().cpu().numpy()
    '''
    
    # Create the target directory if it doesn't exist
    output_dir = 'trained_controller'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
        
    if best_params is not None:
        for name, param in net.named_parameters():
            param.data.copy_(best_params[name])  # Load best parameters
            
        # To make it compatible to matlab indices, we store the index of the neural controllers starting from 1 (i.e. for time step 0, it is step1 here)
        save_model_params(net, f"trained_controller/model_params_step{ind}_rand{sd}_"+ini_str+".mat")
        print(f"Model parameters saved.")

    
    '''
    # Save the inputs and outputs for the test set as 'test_results.mat'
    scipy.io.savemat(os.path.join(output_dir, 'test_results_'+str(ind)+'_rand'+str(sd)+'.mat'), {'inputs_fromNN_test': inputs_fromNN_test_np.T, 'states_test': states_test.T})
    '''
    
    
    finish = time.time()
    print(f"Time used for step {ind} is", finish-start)



finish_all = time.time()
print(f"Total time used is", finish_all-start_all)

"""
# Check the residual norm (ideal if less than 1)
# Evaluation on the training set after training
net.eval()  # Set model to evaluation mode (turns off dropout, etc.)

# Convert the training data to tensors
x_train_tensor = torch.tensor(states_train, dtype=torch.float32)

# Pass the training data through the trained network
u_train = net(x_train_tensor)
y_train = F_Dubin_Car(x_train_tensor, u_train)

# Expand c2 to match y_train's shape
c2_expanded_train = c2.repeat(y_train.shape[0], 1)

# Compute the residual for the training set: f(x^s_k, Π(x^s_k)) - c^2
residual_train = y_train - c2_expanded_train

# Apply the pseudoinverse to the residual on the training set
residual_pinv_train = torch.norm(torch.matmul(G_pseudoinverse, residual_train.T).T, p = float('inf'), dim = 1)

# Extract the value of residual_pinv for the training set
residual_pinv_train_values = residual_pinv_train.detach().cpu().numpy()

# Print or save the extracted values for residual_pinv on the training set
print("Residual_pinv values (training set):")
print(residual_pinv_train_values)

fc1_weights = net.fc1.weight.detach().cpu().numpy()  # Convert to NumPy for easy viewing
print("Check first layer (fc1) weights:")
print(fc1_weights.max())

# Similarly, if you want to check the weights of the second layer (fc2):
fc2_weights = net.fc2.weight.detach().cpu().numpy()  # Convert to NumPy for easy viewing
print("Check second layer (fc2) weights:")
print(fc2_weights.max())

"""